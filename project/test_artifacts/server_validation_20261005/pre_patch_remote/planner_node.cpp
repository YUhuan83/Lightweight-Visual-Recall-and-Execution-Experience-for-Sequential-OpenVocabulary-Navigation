#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <graphnav_msgs/msg/navigation_graph.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <optional>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>
#include "graphnav_planner/planner.hpp"
#include "graphnav_planner/exploration_backoff.hpp"
#include "graphnav_planner/exploration_observation.hpp"
#include "graphnav_planner/exploration_utility.hpp"
#include "graphnav_planner/graph_score_overlay.hpp"

namespace graphnav_planner
{

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode(const rclcpp::NodeOptions& options)
    : Node("planner_node", options)
    , tf_buffer_(this->get_clock())
    , tf_listener_(tf_buffer_, this)
    , planner_(this->get_logger())
  {
    this->declare_parameter("frontier_dist_cost_factor", 2.0);
    this->declare_parameter("goal_dist_cost_factor", 1.0);
    this->declare_parameter("frontier_score_factor", 10.0);
    this->declare_parameter("min_local_frontier_score", 0.4);
    this->declare_parameter("local_frontier_radius", 7.0);
    this->declare_parameter("path_smoothness_period", 10.0);
    this->declare_parameter("physical_risk_factor", 10.0);
    this->declare_parameter("dynamic_risk_factor", 12.0);
    this->declare_parameter("stale_observation_seconds", 5.0);
    this->declare_parameter("low_confidence_factor", 4.0);
    this->declare_parameter("heading_turn_cost_factor", 2.0);

    planner_.frontier_dist_cost_factor_ = this->get_parameter("frontier_dist_cost_factor").as_double();
    planner_.goal_dist_cost_factor_ = this->get_parameter("goal_dist_cost_factor").as_double();
    planner_.frontier_score_factor_ = this->get_parameter("frontier_score_factor").as_double();
    planner_.min_local_frontier_score_ = this->get_parameter("min_local_frontier_score").as_double();
    planner_.local_frontier_radius_ = this->get_parameter("local_frontier_radius").as_double();
    planner_.path_smoothness_period_ = this->get_parameter("path_smoothness_period").as_double();
    planner_.physical_risk_factor_ = this->get_parameter("physical_risk_factor").as_double();
    planner_.dynamic_risk_factor_ = this->get_parameter("dynamic_risk_factor").as_double();
    planner_.stale_observation_seconds_ = this->get_parameter("stale_observation_seconds").as_double();
    planner_.low_confidence_factor_ = this->get_parameter("low_confidence_factor").as_double();
    planner_.heading_turn_cost_factor_ = this->get_parameter("heading_turn_cost_factor").as_double();
    this->declare_parameter("robot_profile_id", "default");
    planner_.robot_profile_id_ = this->get_parameter("robot_profile_id").as_string();

    this->declare_parameter("trav_class", "default");
    trav_class_ = this->get_parameter("trav_class").as_string();
    planner_.set_trav_class(trav_class_);

    this->declare_parameter("goal_radius", 3.0);
    goal_radius_ = this->get_parameter("goal_radius").as_double();
    this->declare_parameter("autonomous_exploration_enabled", false);
    autonomous_exploration_enabled_ =
        this->get_parameter("autonomous_exploration_enabled").as_bool();

    this->declare_parameter("use_compact_score_overlay", false);
    this->declare_parameter("topology_topic", "/spot1/nav_graph");
    const bool compact_scores = this->get_parameter("use_compact_score_overlay").as_bool();
    if (compact_scores)
      topology_sub_ = this->create_subscription<graphnav_msgs::msg::NavigationGraph>(
          this->get_parameter("topology_topic").as_string(), 1,
          [this](const graphnav_msgs::msg::NavigationGraph::ConstSharedPtr msg) { score_overlay_.remember(msg); });
    graph_sub_ = this->create_subscription<graphnav_msgs::msg::NavigationGraph>(
        "~/nav_graph", 1, [this, compact_scores](const graphnav_msgs::msg::NavigationGraph::ConstSharedPtr msg) {
          auto graph = compact_scores ? score_overlay_.combine(*msg) : msg;
          if (!graph) { RCLCPP_WARN(this->get_logger(), "No exact topology snapshot for compact scores; waiting for next observation"); return; }
          this->planner_.update_graph(graph);
          this->latest_graph_ = graph;
          this->latest_graph_header_ = graph->header;
          this->plan_to_goal();
        });
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/goal_pose", 10, [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
          this->goal_pose_ = msg;
          this->explicit_goal_received_ = true;
          this->exploration_target_uuid_.clear();
          this->exploration_backoff_.clear();
          this->plan_to_goal();
        });
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "~/odom", 10, [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) { this->odom_ = msg; });

    clear_goal_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/clear_goal", [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                              std_srvs::srv::Trigger::Response::SharedPtr response) {
          this->goal_pose_.reset();
          this->explicit_goal_received_ = false;
          this->exploration_target_uuid_.clear();
          this->exploration_backoff_.clear();
          response->success = true;
          response->message = "Explicit destination cleared; frontier exploration may resume.";
        });

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("~/path", 10);
    rejection_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/rejected_route_endpoint", 10,
        [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
          const bool exploring = autonomous_exploration_enabled_ && !goal_pose_ && !explicit_goal_received_;
          if (exploration_backoff_.reject(*message, this->get_clock()->now().nanoseconds(), exploring))
          {
            exploration_target_uuid_.clear();
            RCLCPP_INFO(this->get_logger(), "Local vehicle planner rejected exploration route; selecting another frontier");
            this->plan_to_goal();
          }
        });
    route_status_pub_ = this->create_publisher<std_msgs::msg::String>("~/route_status", 10);
    grid_map_debug_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("~/unexplored_space_map", 10);
    scores_debug_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/frontier_scores", 10);
  }

private:
  std::optional<geometry_msgs::msg::PoseStamped> choose_exploration_goal()
  {
    if (!latest_graph_ || latest_graph_->current_node_idx >= latest_graph_->nodes.size())
      return std::nullopt;
    const auto class_it = std::find(latest_graph_->trav_classes.begin(),
                                    latest_graph_->trav_classes.end(), trav_class_);
    if (class_it == latest_graph_->trav_classes.end())
      return std::nullopt;
    const size_t class_idx = std::distance(latest_graph_->trav_classes.begin(), class_it);
    const auto& origin = latest_graph_->nodes[latest_graph_->current_node_idx].pose.position;
    if (!exploration_target_uuid_.empty())
    {
      for (const auto& node : latest_graph_->nodes)
      {
        if (uuid_to_string(node.uuid) != exploration_target_uuid_)
          continue;
        const auto& pos = node.pose.position;
        if (std::hypot(pos.x - origin.x, pos.y - origin.y) <= goal_radius_)
        {
          if (exploration_heading_ && odom_)
          {
            const auto& q = odom_->pose.pose.orientation;
            const double yaw = std::atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z));
            const auto& velocity = odom_->twist.twist.linear;
            if (!exploration_observation_.complete(rclcpp::Time(odom_->header.stamp).nanoseconds(),
                true, yaw, *exploration_heading_, std::hypot(velocity.x, velocity.y)))
            {
              geometry_msgs::msg::PoseStamped held;
              held.header = *latest_graph_header_;
              held.pose.position = pos;
              held.pose.orientation.z = std::sin(*exploration_heading_/2);
              held.pose.orientation.w = std::cos(*exploration_heading_/2);
              return held;
            }
          }
          ++exploration_visits_[exploration_target_uuid_];
          exploration_target_uuid_.clear();
          exploration_heading_.reset();
          exploration_observation_.clear();
        }
        else exploration_observation_.clear();
        break;
      }
    }
    struct Candidate { double utility; size_t index; std::string uuid; };
    std::vector<Candidate> candidates;
    for (size_t i = 0; i < latest_graph_->nodes.size(); ++i)
    {
      const auto& node = latest_graph_->nodes[i];
      if (class_idx >= node.trav_properties.size() || !node.trav_properties[class_idx].is_frontier)
        continue;
      const auto assessment = std::find_if(node.robot_traversability.begin(),
          node.robot_traversability.end(), [this](const auto& item) {
            return item.robot_profile_id == planner_.robot_profile_id_;
          });
      double physical_confidence = 0.0;
      double physical_traversability = 0.0;
      if (assessment != node.robot_traversability.end() && assessment->observed_at.sec > 0)
      {
        const double age = (this->get_clock()->now().nanoseconds() -
                           rclcpp::Time(assessment->observed_at).nanoseconds()) * 1e-9;
        if (age >= 0.0 && age <= planner_.stale_observation_seconds_ &&
            std::isfinite(assessment->confidence) && assessment->confidence >= 0.0f &&
            assessment->confidence <= 1.0f)
        {
          physical_confidence = assessment->confidence;
          physical_traversability = assessment->physical_traversability;
          if (physical_confidence >= 0.7 &&
              (!assessment->feasible || !std::isfinite(physical_traversability) ||
               physical_traversability <= 0.0 || !std::isfinite(assessment->dynamic_risk) ||
               assessment->dynamic_risk >= 1.0f)) continue;
        }
      }
      const auto& pos = node.pose.position;
      if (exploration_backoff_.blocked(pos, this->get_clock()->now().nanoseconds(),
                                      std::max(rejected_frontier_radius_m, goal_radius_))) continue;
      const double distance = std::hypot(pos.x - origin.x, pos.y - origin.y);
      if (distance <= goal_radius_)
        continue;
      double score = 0.3;
      for (const auto& property : node.properties)
        if (property.key == "frontier_scores" && !property.value.empty())
          score = *std::max_element(property.value.begin(), property.value.end());
      if (!std::isfinite(score) || score <= 1e-4)
        continue;
      if (!node.heading_candidates.empty())
      {
        double best_utility = -std::numeric_limits<double>::infinity();
        for (const auto& heading : node.heading_candidates)
          if (heading.traversable)
            best_utility = std::max(best_utility,
                                    static_cast<double>(heading.predicted_utility));
        if (!std::isfinite(best_utility))
          continue;
        score = best_utility;
      }
      const std::string uuid = uuid_to_string(node.uuid);
      const int visits = exploration_visits_[uuid];
      const double utility = exploration_candidate_utility(
          score, distance, visits, physical_confidence, physical_traversability,
          uuid == exploration_target_uuid_);
      candidates.push_back({utility, i, uuid});
    }
    if (candidates.empty())
      return std::nullopt;
    const auto selected = std::max_element(candidates.begin(), candidates.end(),
        [](const auto& a, const auto& b) { return a.utility < b.utility; });
    if (exploration_target_uuid_ != selected->uuid)
    {
      exploration_heading_ = exploration_heading(latest_graph_->nodes[selected->index]);
      exploration_observation_.clear();
    }
    exploration_target_uuid_ = selected->uuid;
    geometry_msgs::msg::PoseStamped goal;
    goal.header = *latest_graph_header_;
    goal.pose.position = latest_graph_->nodes[selected->index].pose.position;
    const double heading = exploration_heading_.value_or(0.);
    goal.pose.orientation.z = std::sin(heading/2);
    goal.pose.orientation.w = std::cos(heading/2);
    return goal;
  }

  void plan_to_goal()
  {
    if (!latest_graph_header_)
      return;
    const bool exploring = !goal_pose_ && autonomous_exploration_enabled_ &&
        !explicit_goal_received_;
    if (goal_pose_ || exploring)
    {
      auto exploration_goal = exploring ? choose_exploration_goal() : std::nullopt;
      if (exploring && !exploration_goal)
        return;
      geometry_msgs::msg::PoseStamped goal = exploring ? *exploration_goal : *goal_pose_;
      goal.header.stamp = latest_graph_header_->stamp;
      geometry_msgs::msg::PoseStamped goal_in_graph_frame;
      try
      {
        goal_in_graph_frame = exploring ? goal :
            tf_buffer_.transform(goal, latest_graph_header_->frame_id, tf2::durationFromSec(0.1));
      }
      catch (const tf2::TransformException& ex)
      {
        RCLCPP_WARN(this->get_logger(), "Could not transform goal pose to graph frame: %s", ex.what());
        return;
      }
      Eigen::Vector3d goal_vec(goal_in_graph_frame.pose.position.x, goal_in_graph_frame.pose.position.y,
                               goal_in_graph_frame.pose.position.z);
      auto path = planner_.plan_to_goal(goal_vec, goal_radius_, this->get_clock()->now());
      if (exploring && path.empty())
      {
        ++exploration_visits_[exploration_target_uuid_];
        exploration_target_uuid_.clear();
        return;
      }
      nav_msgs::msg::Path path_msg;
      path_msg.header = *latest_graph_header_;
      path_msg.header.stamp = this->get_clock()->now();
      path_msg.poses.resize(path.size());
      for (size_t i = 0; i < path.size(); i++)
      {
        path_msg.poses[i].pose.position.x = path[i].x();
        path_msg.poses[i].pose.position.y = path[i].y();
        path_msg.poses[i].pose.position.z = path[i].z();
        double heading = i ? 2.0 * std::atan2(path_msg.poses[i-1].pose.orientation.z,
                                                path_msg.poses[i-1].pose.orientation.w) : 0.0;
        if (i + 1 < path.size() && (path[i+1].head<2>() - path[i].head<2>()).norm() > 1e-6)
          heading = std::atan2(path[i+1].y() - path[i].y(), path[i+1].x() - path[i].x());
        path_msg.poses[i].header = path_msg.header;
        path_msg.poses[i].pose.orientation.z = std::sin(heading / 2.0);
        path_msg.poses[i].pose.orientation.w = std::cos(heading / 2.0);
      }
      std_msgs::msg::String status;
      std::ostringstream encoded;
      encoded << std::setprecision(17) << "{\"stamp_ns\":"
              << rclcpp::Time(path_msg.header.stamp).nanoseconds()
              << ",\"goal\":[" << goal_vec.x() << "," << goal_vec.y()
              << "],\"complete\":" << (planner_.route_complete() ? "true" : "false")
              << ",\"mode\":\"" << (path.empty() ? "none" : planner_.route_complete() ? "known" : "frontier")
              << "\"";
      if (exploring && exploration_heading_ && planner_.route_complete())
        encoded << ",\"observation_heading\":" << *exploration_heading_;
      encoded << "}";
      status.data = encoded.str();
      route_status_pub_->publish(status);
      if (exploring && !path_msg.poses.empty())
        exploration_backoff_.remember(path_msg.header, path_msg.poses.back().pose.position,
                                      goal_in_graph_frame.pose.position, exploration_target_uuid_);
      path_pub_->publish(path_msg);
      if (grid_map_debug_pub_->get_subscription_count() > 0)
      {
        grid_map_msgs::msg::GridMap grid_map_msg = planner_.get_unexplored_debug_map();
        grid_map_msg.header = *latest_graph_header_;
        grid_map_debug_pub_->publish(grid_map_msg);
      }
      if (scores_debug_pub_->get_subscription_count() > 0)
      {
        visualization_msgs::msg::MarkerArray marker_array = planner_.get_score_visualization(
          this->get_clock()->now(), latest_graph_header_->frame_id, true);
        scores_debug_pub_->publish(marker_array);
      }
      // goal_radius_ connects the destination to the navigation graph. It is
      // not a task completion tolerance: a nearby remembered object can still
      // require a final approach and visual confirmation. Keep explicit goals
      // until the owner replaces them or calls clear_goal on a query change.
    }
  }

  rclcpp::Subscription<graphnav_msgs::msg::NavigationGraph>::SharedPtr graph_sub_;
  rclcpp::Subscription<graphnav_msgs::msg::NavigationGraph>::SharedPtr topology_sub_;
  GraphScoreOverlay score_overlay_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rejection_sub_;
  ExplorationBackoff exploration_backoff_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr route_status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_goal_service_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_debug_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr scores_debug_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  geometry_msgs::msg::PoseStamped::ConstSharedPtr goal_pose_;
  graphnav_msgs::msg::NavigationGraph::ConstSharedPtr latest_graph_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  std::optional<std_msgs::msg::Header> latest_graph_header_;
  double goal_radius_;
  bool autonomous_exploration_enabled_ = false;
  bool explicit_goal_received_ = false;
  std::string trav_class_;
  std::string exploration_target_uuid_;
  std::optional<double> exploration_heading_;
  ExplorationObservation exploration_observation_;
  std::unordered_map<std::string, int> exploration_visits_;

  Planner planner_;
};

}  // namespace graphnav_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(graphnav_planner::PlannerNode)
