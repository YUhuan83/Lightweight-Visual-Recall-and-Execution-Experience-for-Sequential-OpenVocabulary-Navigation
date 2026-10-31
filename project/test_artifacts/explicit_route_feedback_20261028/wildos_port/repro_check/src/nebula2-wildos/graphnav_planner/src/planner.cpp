#include <rclcpp/rclcpp.hpp>
#include <graaflib/graph.h>
#include <graaflib/algorithm/shortest_path/dijkstra_shortest_path.h>
#include <unordered_map>
#include <map>
#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>

#include "graphnav_planner/planner.hpp"

namespace graphnav_planner
{

namespace
{
bool measured_physical_violation(const std::vector<graphnav_msgs::msg::KeyValue>& properties)
{
  return std::any_of(properties.begin(),properties.end(),[](const auto& property) {
    return property.key=="physical_violation_observed" && property.value.size()==1 && property.value[0]==1.f;
  });
}

double node_property(const graphnav_msgs::msg::Node& node, const std::string& key, double fallback)
{
  for (const auto& property : node.properties)
  {
    if (property.key == key && !property.value.empty())
      return property.value.front();
  }
  return fallback;
}

const graphnav_msgs::msg::HeadingCandidate* best_heading_candidate(
    const graphnav_msgs::msg::Node& node, double heading)
{
  const graphnav_msgs::msg::HeadingCandidate* best = nullptr;
  double best_delta = std::numeric_limits<double>::max();
  for (const auto& candidate : node.heading_candidates)
  {
    if (!candidate.traversable)
      continue;
    const double delta = std::abs(std::atan2(
        std::sin(static_cast<double>(candidate.heading) - heading),
        std::cos(static_cast<double>(candidate.heading) - heading)));
    if (delta < best_delta)
    {
      best_delta = delta;
      best = &candidate;
    }
  }
  return best;
}

template<typename Assessment>
bool assessment_fresh(const Assessment* assessment, rclcpp::Time now, double ttl)
{
  if (assessment == nullptr || assessment->observed_at.sec <= 0 ||
      assessment->observed_at.nanosec >= 1000000000u ||
      !std::isfinite(assessment->confidence) || assessment->confidence < 0.7f ||
      assessment->confidence > 1.0f) return false;
  const double age = (now.nanoseconds() - rclcpp::Time(assessment->observed_at).nanoseconds()) * 1e-9;
  return age >= 0.0 && age <= ttl;
}

bool profile_node_feasible(const graphnav_msgs::msg::Node& node, const std::string& profile_id)
{
  for (const auto& assessment : node.robot_traversability)
    if (assessment.robot_profile_id == profile_id)
      return !std::isfinite(assessment.confidence) || assessment.confidence < 0.7f ||
          assessment.confidence > 1.0f || assessment.observed_at.sec <= 0 || assessment.feasible;
  return true;  // Support graphs published by older mapper versions.
}

const graphnav_msgs::msg::RobotNodeTraversability* profile_node_assessment(
    const graphnav_msgs::msg::Node& node, const std::string& profile_id)
{
  for (const auto& assessment : node.robot_traversability)
    if (assessment.robot_profile_id == profile_id)
      return &assessment;
  return nullptr;
}

const graphnav_msgs::msg::RobotEdgeTraversability* profile_edge_assessment(
    const graphnav_msgs::msg::Edge& edge, const std::string& profile_id)
{
  for (const auto& assessment : edge.robot_traversability)
    if (assessment.robot_profile_id == profile_id)
      return &assessment;
  return nullptr;
}

bool profile_edge_feasible(const graphnav_msgs::msg::Edge& edge, const std::string& profile_id)
{
  for (const auto& assessment : edge.robot_traversability)
    if (assessment.robot_profile_id == profile_id)
      return assessment.physical_traversability > 0.0f;
  return true;
}
}

Planner::Planner(rclcpp::Logger logger) : logger_(logger)
{
}

void Planner::set_trav_class(std::string trav_class)
{
  trav_class_ = trav_class;
}

void Planner::update_graph(graphnav_msgs::msg::NavigationGraph::ConstSharedPtr graph)
{
  graph_ = decltype(graph_){};
  for (size_t i = 0; i < graph->nodes.size(); i++)
  {
    graphnav_msgs::msg::Node node = graph->nodes[i];
    const auto& position = node.pose.position;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z))
    {
      unexplored_space_map_.reset();
      frontier_scores_.clear();
      return;
    }
    graph_.add_vertex(node, i);
  }
  if (graph->nodes.empty() || graph->current_node_idx >= graph->nodes.size())
  {
    unexplored_space_map_.reset();
    frontier_scores_.clear();
    return;
  }
  auto trav_class_it = std::find(graph->trav_classes.begin(), graph->trav_classes.end(), trav_class_);
  if (trav_class_it == graph->trav_classes.end())
  {
    RCLCPP_WARN(logger_, "Traversability class %s not found in graph", trav_class_.c_str());
    trav_class_idx_ = 0;
    unexplored_space_map_.reset();
    frontier_scores_.clear();
    return;
  }
  trav_class_idx_ = std::distance(graph->trav_classes.begin(), trav_class_it);
  for (auto edge : graph->edges)
  {
    if (edge.from_idx >= graph->nodes.size() || edge.to_idx >= graph->nodes.size() ||
        trav_class_idx_ >= edge.traversability.size()) continue;
    const auto& from_position = graph->nodes[edge.from_idx].pose.position;
    const auto& to_position = graph->nodes[edge.to_idx].pose.position;
    const Eigen::Vector2d from_xy(from_position.x, from_position.y);
    const Eigen::Vector2d to_xy(to_position.x, to_position.y);
    constexpr double rejected_edge_endpoint_tolerance_m = 0.25;
    const bool recently_rejected = std::any_of(rejected_edges_.begin(), rejected_edges_.end(),
        [&](const auto& rejected) {
          return ((from_xy - rejected.first).norm() <= rejected_edge_endpoint_tolerance_m &&
                  (to_xy - rejected.second).norm() <= rejected_edge_endpoint_tolerance_m) ||
                 ((from_xy - rejected.second).norm() <= rejected_edge_endpoint_tolerance_m &&
                  (to_xy - rejected.first).norm() <= rejected_edge_endpoint_tolerance_m);
        });
    if (recently_rejected) continue;
    bool traversed = false;
    for (const auto& property : edge.traversability[trav_class_idx_].properties)
      if (property.key == "traversed" && !property.value.empty() && property.value.front() > 0.5f)
        traversed = true;
    const auto* edge_assessment = profile_edge_assessment(edge, robot_profile_id_);
    const auto fresh = [&](const auto* assessment) {
      return assessment_fresh(assessment, rclcpp::Time(graph->header.stamp), stale_observation_seconds_);
    };
    bool endpoints_feasible = true;
    bool measured_endpoint_blocked = false;
    for (const auto index : {edge.from_idx, edge.to_idx}) {
      const auto* assessment = profile_node_assessment(graph->nodes[index], robot_profile_id_);
      if (fresh(assessment) && !assessment->feasible) endpoints_feasible = false;
      if (fresh(assessment) && !assessment->feasible && measured_physical_violation(graph->nodes[index].properties))
        measured_endpoint_blocked=true;
    }
    const bool fresh_dynamic_assessment = fresh(edge_assessment);
    const bool hard_blocked = fresh_dynamic_assessment &&
        edge_assessment->physical_traversability <= 0.0f;
    const bool measured_edge_blocked=hard_blocked && measured_physical_violation(edge.properties);
    // A driven edge remains stored topology. Fresh measured danger is not
    // execution permission, including when older visits previously succeeded.
    if (!measured_endpoint_blocked && !measured_edge_blocked &&
        (traversed || (endpoints_feasible && !hard_blocked)))
    {
      double weight = edge.traversability[trav_class_idx_].traversability_cost;
      if (!std::isfinite(weight) || weight < 0.0) continue;
      if (edge_assessment != nullptr && fresh_dynamic_assessment)
      {
        weight += dynamic_risk_factor_ * (std::isfinite(edge_assessment->dynamic_risk) ?
            std::clamp(static_cast<double>(edge_assessment->dynamic_risk), 0.0, 1.0) : 1.0);
        weight += low_confidence_factor_ * (1.0 - edge_assessment->confidence);
      }
      else if (edge_assessment != nullptr)
      {
        // A stale dynamic estimate should not erase a physically traversable
        // edge. Penalize uncertainty until the mapper refreshes it.
        weight += low_confidence_factor_;
      }
      if (traversed && (!endpoints_feasible || hard_blocked)) weight += physical_risk_factor_;
      if (!std::isfinite(weight) || weight < 0.0) continue;
      graph_.add_edge(edge.from_idx, edge.to_idx, weight);
    }
  }
  current_node_idx_ = graph->current_node_idx;
  unexplored_space_map_ = compute_unexplored_space_map();
}

UnexploredSpaceMap Planner::compute_unexplored_space_map()
{
  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();
  for (auto& [id, node] : graph_.get_vertices())
  {
    const auto& pos = node.pose.position;
    min_x = std::min(min_x, pos.x);
    max_x = std::max(max_x, pos.x);
    min_y = std::min(min_y, pos.y);
    max_y = std::max(max_y, pos.y);
  }
  double resolution = 1.0;
  double margin = 10.0;
  const auto& current = graph_.get_vertex(current_node_idx_).pose.position;
  min_x = std::max(min_x, current.x - 1000.0); max_x = std::min(max_x, current.x + 1000.0);
  min_y = std::max(min_y, current.y - 1000.0); max_y = std::min(max_y, current.y + 1000.0);
  UnexploredSpaceMap unexplored_map(min_x, max_x, min_y, max_y, margin, resolution);
  for (auto& [id, node] : graph_.get_vertices())
  {
    if (trav_class_idx_ < node.trav_properties.size())
    {
      double explored_radius = node.trav_properties[trav_class_idx_].explored_radius;
      if (explored_radius > 0)
      {
        const auto& pos = node.pose.position;
        unexplored_map.mark_explored(pos.x, pos.y, explored_radius);
      }
    }
  }
  return unexplored_map;
}

std::vector<Eigen::Vector3d> Planner::plan_to_goal(Eigen::Vector3d& goal, double goal_radius, rclcpp::Time current_time,
    const std::function<bool(const Eigen::Vector3d&)>& endpoint_is_postponed)
{
  route_complete_ = false;
  if (!unexplored_space_map_ || !goal.allFinite() || !std::isfinite(goal_radius) || goal_radius <= 0.0)
  {
    return {};
  }
  graphnav_msgs::msg::Node virtual_goal_node;
  virtual_goal_node.pose.position.x = goal.x();
  virtual_goal_node.pose.position.y = goal.y();
  virtual_goal_node.pose.position.z = goal.z();
  graaf::vertex_id_t virtual_goal = graph_.add_vertex(virtual_goal_node);

  frontier_scores_.clear();
  for (const auto& [id, node] : graph_.get_vertices())
  {
    if (id == virtual_goal) continue;
    bool visited = false;
    if (trav_class_idx_ < node.trav_properties.size())
      for (const auto& property : node.trav_properties[trav_class_idx_].properties)
        if (property.key == "visited" && !property.value.empty() && property.value.front() > 0.5f)
          visited = true;
    const auto* physical = profile_node_assessment(node, robot_profile_id_);
    const bool fresh = assessment_fresh(physical, current_time, stale_observation_seconds_);
    if (fresh && !physical->feasible && measured_physical_violation(node.properties)) continue;
    if (!visited && fresh && (!physical->feasible || physical->dynamic_risk >= 1.0f)) continue;
    const auto& p = node.pose.position;
    const double gap = (Eigen::Vector3d(p.x, p.y, p.z) - goal).norm();
    const double cost = goal_dist_cost_factor_ * gap;
    if (gap < goal_radius && std::isfinite(cost) && cost >= 0.0)
      graph_.add_edge(id, virtual_goal, cost);
  }
  auto known_path = graaf::algorithm::dijkstra_shortest_path(graph_, current_node_idx_, virtual_goal);
  if (known_path)
  {
    std::vector<Eigen::Vector3d> points;
    for (const auto id : known_path->vertices) {
      const auto& pos = graph_.get_vertex(id).pose.position;
      points.emplace_back(pos.x, pos.y, pos.z);
    }
    route_complete_ = true;
    latest_frontier_.reset();
    latest_frontier_time_.reset();
    graph_.remove_vertex(virtual_goal);
    return points;
  }
  unexplored_space_map_->compute_distance_from(goal.x(), goal.y());

  std::vector<graaf::vertex_id_t> local_scored_frontiers;
  std::unordered_map<graaf::vertex_id_t, Eigen::Vector3d> frontier_endpoints;

  for (const auto& [id, node] : graph_.get_vertices())
  {
    if (id == virtual_goal) continue;
    bool visited = false;
    if (trav_class_idx_ < node.trav_properties.size())
      for (const auto& property : node.trav_properties[trav_class_idx_].properties)
        if (property.key == "visited" && !property.value.empty() && property.value.front() > 0.5f)
          visited = true;
    const auto* physical = profile_node_assessment(node, robot_profile_id_);
    const bool fresh = assessment_fresh(physical, current_time, stale_observation_seconds_);
    const bool fresh_blocked = fresh && !physical->feasible;
    if (fresh_blocked && measured_physical_violation(node.properties)) continue;
    if (fresh_blocked && !visited) continue;
    const auto* node_assessment = physical;
    if (!visited && fresh && physical->dynamic_risk >= 1.0f) continue;
    Eigen::Vector3d node_pos;
    node_pos.x() = node.pose.position.x;
    node_pos.y() = node.pose.position.y;
    node_pos.z() = node.pose.position.z;

    if (!fresh_blocked && trav_class_idx_ < node.trav_properties.size() && node.trav_properties[trav_class_idx_].is_frontier)
    {
      double frontier_path_distance = std::numeric_limits<double>::max();
      double frontier_score = -1.0;
      double frontier_cost = std::numeric_limits<double>::max();

      std::optional<Eigen::Vector3d> frontier_endpoint;
      for (const auto& frontier_pt : node.trav_properties[trav_class_idx_].frontier_points)
      {
        if (!std::isfinite(frontier_pt.x) || !std::isfinite(frontier_pt.y) ||
            !std::isfinite(frontier_pt.z)) continue;
        if (endpoint_is_postponed && endpoint_is_postponed(
            Eigen::Vector3d(frontier_pt.x,frontier_pt.y,frontier_pt.z))) continue;
        double d = unexplored_space_map_->query_distance_to(frontier_pt.x, frontier_pt.y);
        if (!std::isfinite(d))
          d = std::hypot(frontier_pt.x - goal.x(), frontier_pt.y - goal.y());
        const bool tied_before = frontier_endpoint && d == frontier_path_distance &&
            std::tuple(frontier_pt.x, frontier_pt.y, frontier_pt.z) <
            std::tuple(frontier_endpoint->x(), frontier_endpoint->y(), frontier_endpoint->z());
        if (!frontier_endpoint || d < frontier_path_distance || tied_before) {
          frontier_path_distance = d;
          frontier_endpoint = Eigen::Vector3d(frontier_pt.x, frontier_pt.y, frontier_pt.z);
        }
      }

      if (!frontier_endpoint) continue;
      frontier_endpoints.emplace(id, *frontier_endpoint);

      // compute heading to the goal
      Eigen::Vector3d heading = goal - node_pos;
      const auto* heading_candidate = best_heading_candidate(
          node, std::atan2(heading.y(), heading.x()));
      if (!node.heading_candidates.empty() && heading_candidate == nullptr)
        continue;
      bool has_frontier_scores = false;
      double cur_frontier_dist_cost_factor = std::numeric_limits<double>::max();
      for (const auto& kv: node.properties)
      {
        if (kv.key == "frontier_scores")
        {
          if (kv.value.empty())
            continue;
          int num_bins = kv.value.size();
          double angle_per_bin = 2 * M_PI / num_bins;
          double heading_angle = std::atan2(heading.y(), heading.x());
          if (heading_angle < 0)
            heading_angle += 2 * M_PI;
          int best_bin = static_cast<int>(std::round(heading_angle / angle_per_bin)) % num_bins;
          if (!std::isfinite(kv.value[best_bin])) continue;
          has_frontier_scores = true;
          frontier_score = std::clamp(static_cast<double>(kv.value[best_bin]), 1e-4, 1.0);
          cur_frontier_dist_cost_factor = 1.0 - frontier_score_factor_ * std::log(frontier_score);

          if (latest_frontier_){
            double distance_to_latest_frontier = (node_pos - *latest_frontier_).norm();
            if (distance_to_latest_frontier < local_frontier_radius_ && frontier_score > min_local_frontier_score_){
              local_scored_frontiers.push_back(id);
            }
          }
          
          // if (frontier_score>0 && frontier_path_distance == std::numeric_limits<double>::max()){
          //   RCLCPP_INFO(logger_, "Node %s frontier score in best bin %d is %f, setting frontier_dist_cost_factor to %f",
          //              uuid_to_string(node.uuid).c_str(), best_bin, frontier_score, cur_frontier_dist_cost_factor);
          // }
        }
      }
      // Sparse score overlays are per-node. An unscored frontier must not
      // disable valid scores on subsequent nodes in unordered graph traversal.
      if (!has_frontier_scores)
      {
        frontier_cost = frontier_path_distance * frontier_dist_cost_factor_;
        // RCLCPP_WARN(logger_, "Node %ld is a frontier but has no frontier_scores property", id);
      
        // add to local frontier based on just distance
        if (latest_frontier_){
            double distance_to_latest_frontier = (node_pos - *latest_frontier_).norm();
            if (distance_to_latest_frontier < local_frontier_radius_){
              local_scored_frontiers.push_back(id);
            }
          }
      }
      else
      {
        frontier_cost = frontier_path_distance * cur_frontier_dist_cost_factor;
      }
      if (heading_candidate != nullptr)
      {
        frontier_cost += heading_turn_cost_factor_ * heading_candidate->motion_cost;
        frontier_cost -= heading_turn_cost_factor_ * heading_candidate->predicted_utility;
      }
      frontier_cost += physical_risk_factor_ * node_property(node, "physical_risk", 0.0);
      if (node_assessment != nullptr)
      {
        frontier_cost += dynamic_risk_factor_ * node_assessment->dynamic_risk;
        frontier_cost += low_confidence_factor_ * (1.0 - node_assessment->confidence);
      }
      if (!std::isfinite(frontier_cost)) continue;
      frontier_cost = std::max(0.0, frontier_cost);
      frontier_scores_[id] = std::make_pair(node, std::make_pair(frontier_score, frontier_cost));
    }


  }
  

  // if current_time is within path_smoothness_period_ of latest_frontier_time_, prefer local frontiers
  bool use_local_frontiers = false;
  if (!local_scored_frontiers.empty())
  {
    if (latest_frontier_time_ && (current_time - *latest_frontier_time_).seconds() < path_smoothness_period_)
    {
      for (auto id: local_scored_frontiers)
      {
        double frontier_cost = frontier_scores_[id].second.second;
        graph_.add_edge(id, virtual_goal, frontier_cost);
        use_local_frontiers = true;
      }
    }
    else
    {
      latest_frontier_time_ = current_time;
    }
  }  
  
  if (!use_local_frontiers)
  {
    for (auto& [id, score_pair] : frontier_scores_)
    {
      double frontier_cost = score_pair.second.second;
      graph_.add_edge(id, virtual_goal, frontier_cost);
    }
    latest_frontier_time_ = current_time;
  }

  auto path = graaf::algorithm::dijkstra_shortest_path(graph_, current_node_idx_, virtual_goal);
  std::vector<Eigen::Vector3d> path_points;
  bool has_frontier_in_path = false;
  if (path)
  {
    size_t idx = 0;
    for (const auto& node_id : path->vertices)
    {
      if (node_id == virtual_goal) continue; // Estimated unseen tail is not an executable route.
      const auto& node = graph_.get_vertex(node_id);
      const auto& pos = node.pose.position;
      path_points.push_back(Eigen::Vector3d(pos.x, pos.y, pos.z));
      if (idx == path->vertices.size() - 2 && trav_class_idx_ < node.trav_properties.size() &&
          node.trav_properties[trav_class_idx_].is_frontier)
      {
        // Execute the same finite frontier exit used to estimate the unseen
        // tail. Averaging disjoint exits invents an unscored location which
        // can point backwards or lie between traversable regions.
        path_points.push_back(frontier_endpoints.at(node_id));

        latest_frontier_ = Eigen::Vector3d(node.pose.position.x, node.pose.position.y, node.pose.position.z);
        has_frontier_in_path = true;
      }
      idx++;
    }
  }
  if (!has_frontier_in_path)
  {
    latest_frontier_.reset();
    RCLCPP_WARN(logger_, "NO FRONTIER IN PATH!!!!!");
  }

  graph_.remove_vertex(virtual_goal);
  return path_points;
}

visualization_msgs::msg::MarkerArray Planner::get_score_visualization(const rclcpp::Time& stamp,
                                                                      std::string frame_id,
                                                                      bool with_id_text) const
{
  visualization_msgs::msg::MarkerArray markers;

  // delete all previous markers
  visualization_msgs::msg::Marker delete_all_infcost;
  delete_all_infcost.action = visualization_msgs::msg::Marker::DELETEALL;
  delete_all_infcost.header.frame_id = frame_id;
  delete_all_infcost.header.stamp = stamp;
  delete_all_infcost.ns = "frontier_inf_cost";
  delete_all_infcost.id = 0;
  markers.markers.push_back(delete_all_infcost);

  visualization_msgs::msg::Marker delete_all_idtext;
  delete_all_idtext.action = visualization_msgs::msg::Marker::DELETEALL;
  delete_all_idtext.header.frame_id = frame_id;
  delete_all_idtext.header.stamp = stamp;
  delete_all_idtext.ns = "frontier_id_text";
  delete_all_idtext.id = 0;
  markers.markers.push_back(delete_all_idtext);

  //visualize ring of local_frontier_radius around latest_frontier_
  if (latest_frontier_) {
    visualization_msgs::msg::Marker frontier_marker;
    frontier_marker.header.frame_id = frame_id;
    frontier_marker.header.stamp = stamp;
    frontier_marker.ns = "local_frontier";
    frontier_marker.id = 0;
    frontier_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    frontier_marker.action = visualization_msgs::msg::Marker::ADD;
    frontier_marker.pose.position.x = latest_frontier_->x();
    frontier_marker.pose.position.y = latest_frontier_->y();
    frontier_marker.pose.position.z = latest_frontier_->z();
    frontier_marker.scale.x = local_frontier_radius_ * 2;
    frontier_marker.scale.y = local_frontier_radius_ * 2;
    frontier_marker.scale.z = 0.1;
    frontier_marker.color.a = 0.5;
    frontier_marker.color.r = 0.0;
    frontier_marker.color.g = 1.0;
    frontier_marker.color.b = 0.0;
    markers.markers.push_back(frontier_marker);
  }
  else{
    // delete
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    delete_marker.header.frame_id = frame_id;
    delete_marker.header.stamp = stamp;
    delete_marker.ns = "local_frontier";
    delete_marker.id = 0;
    markers.markers.push_back(delete_marker);
  }

  visualization_msgs::msg::Marker delete_frontier_time;
  delete_frontier_time.header.frame_id = frame_id;
  delete_frontier_time.header.stamp = stamp;
  delete_frontier_time.ns = "local_frontier_time";
  delete_frontier_time.id = 0;
  delete_frontier_time.action = visualization_msgs::msg::Marker::DELETE;
  markers.markers.push_back(delete_frontier_time);
  // add text for time_diff
  if (latest_frontier_time_ && latest_frontier_)
  {
    double time_diff = (stamp - *latest_frontier_time_).seconds();
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = frame_id;
    text_marker.header.stamp = stamp;
    text_marker.ns = "local_frontier_time";
    text_marker.id = 0;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position.x = latest_frontier_->x() + local_frontier_radius_;
    text_marker.pose.position.y = latest_frontier_->y() + local_frontier_radius_;
    text_marker.pose.position.z = latest_frontier_->z() + 1.0;  // raise text above the node
    text_marker.scale.z = 1.5;           // only scale.z is used for text
    text_marker.color.a = 1.0;
    text_marker.color.r = 0.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    std::ostringstream ss;
    ss << "Exploitation\nTimeout: " << path_smoothness_period_ - time_diff;
    text_marker.text = ss.str();
    markers.markers.push_back(text_marker);
  }

  // visualize cubes for nodes with inf frontier cost
  int id_counter = 1;

  // normalize costs to [0,1] for visualization
  double min_val = 30.0;
  double max_val = 400.0;

  for (const auto& [id, score_pair] : frontier_scores_)
  {
    const auto& node = score_pair.first;
    double frontier_score = score_pair.second.first;
    double frontier_cost = score_pair.second.second;
    // if (frontier_cost == std::numeric_limits<double>::infinity() && frontier_score > 0)
    if (true)
    {
      double norm_cost = (frontier_cost - min_val) / (max_val - min_val);
      norm_cost = std::min(1.0, std::max(0.0, norm_cost));
      std_msgs::msg::ColorRGBA color = colormapJet(norm_cost);
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame_id;
      marker.header.stamp = stamp;
      marker.ns = "frontier_inf_cost";
      marker.id = id_counter;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = node.pose;
      marker.scale.x = 0.5;
      marker.scale.y = 0.5;
      marker.scale.z = 0.5;
      // marker.color.a = 0.5;
      // marker.color.r = 1.0;
      // marker.color.g = 0.0;
      // marker.color.b = 0.0;
      marker.color = color;
      markers.markers.push_back(marker);
    }
    if (with_id_text)
    {
      visualization_msgs::msg::Marker text_marker;
      text_marker.header.frame_id = frame_id;
      text_marker.header.stamp = stamp;
      text_marker.ns = "frontier_id_text";
      text_marker.id = id_counter;
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::msg::Marker::ADD;
      text_marker.pose = node.pose;
      text_marker.pose.position.z += 0.1;  // raise text above the node
      text_marker.scale.z = 0.5;           // only scale.z is used for text
      text_marker.color.a = 1.0;
      text_marker.color.r = 1.0;
      text_marker.color.g = 1.0;
      text_marker.color.b = 1.0;
      std::ostringstream ss;
      // ss << "Score: " << frontier_score << " Cost: " << frontier_cost;
      // upto 2 decimal places
      ss << std::fixed << std::setprecision(2);
      ss << frontier_score << "/" << frontier_cost;
      text_marker.text = ss.str();
      markers.markers.push_back(text_marker);
    }
    id_counter++;
  }
  // }
  return markers;
}


}  // namespace graphnav_planner
