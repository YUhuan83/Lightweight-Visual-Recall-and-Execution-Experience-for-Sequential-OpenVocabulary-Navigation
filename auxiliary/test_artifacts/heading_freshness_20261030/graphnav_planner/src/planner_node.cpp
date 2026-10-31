#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <graphnav_msgs/msg/navigation_graph.hpp>
#include <graphnav_msgs/msg/exploration_choice.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <optional>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <stdexcept>
#include <unordered_map>
#include <vector>
#include "graphnav_planner/planner.hpp"
#include "graphnav_planner/exploration_backoff.hpp"
#include "graphnav_planner/exploration_observation.hpp"
#include "graphnav_planner/exploration_target.hpp"
#include "graphnav_planner/exploration_utility.hpp"
#include "graphnav_planner/graph_score_overlay.hpp"
#include "graphnav_planner/destination_fence.hpp"

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
    rcl_interfaces::msg::ParameterDescriptor frozen_profile;
    frozen_profile.read_only = true;
    frozen_profile.description = "Applied at startup; restart to change the robot profile";
    this->declare_parameter<std::string>("robot_profile_id", "default", frozen_profile);
    planner_.robot_profile_id_ = this->get_parameter("robot_profile_id").as_string();
    this->declare_parameter<double>("missing_frontier_timeout_seconds", 10., frozen_profile);
    missing_frontier_timeout_seconds_ = this->get_parameter("missing_frontier_timeout_seconds").as_double();
    if (!std::isfinite(missing_frontier_timeout_seconds_) || missing_frontier_timeout_seconds_ <= 0.)
      throw std::invalid_argument("missing_frontier_timeout_seconds must be finite and positive");

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
          this->planner_.set_rejected_edges(exploration_backoff_.active_edges(
              this->get_clock()->now().nanoseconds()));
          this->planner_.update_graph(graph);
          this->latest_graph_ = graph;
          this->latest_graph_header_ = graph->header;
          this->plan_to_goal();
        });
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/goal_pose", 10, [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
          const auto& stamp = msg->header.stamp;
          if (stamp.sec < 0 || stamp.nanosec >= 1000000000u ||
              !destination_fence_.accept(int64_t(stamp.sec)*1000000000LL+stamp.nanosec,
                                        this->get_clock()->now().nanoseconds()))
          {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                "Ignoring destination outside the current clear/time fence");
            return;
          }
          this->goal_pose_ = msg;
          this->explicit_goal_received_ = true;
          this->retire_exploration("explicit_destination");
          // A new observation/recall destination is not a new search query.
          this->planner_.set_rejected_edges(exploration_backoff_.active_edges(
              this->get_clock()->now().nanoseconds()));
          if (this->latest_graph_) this->planner_.update_graph(this->latest_graph_);
          this->plan_to_goal();
        });
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "~/odom", 10, [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) { this->odom_ = msg; });

    clear_goal_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/clear_goal", [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                              std_srvs::srv::Trigger::Response::SharedPtr response) {
          this->clear_destination(true, *response);
        });
    release_goal_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/release_goal", [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                                std_srvs::srv::Trigger::Response::SharedPtr response) {
          this->clear_destination(false, *response);
        });

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("~/path", 10);
    rejection_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/rejected_route_endpoint", 10,
        [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
          const bool exploring = autonomous_exploration_enabled_ && !goal_pose_ && !explicit_goal_received_;
          const int64_t now_ns = this->get_clock()->now().nanoseconds();
          const auto identity = exploration_backoff_.identity(*message, now_ns);
          if (!exploration_target_.active() || !identity ||
              *identity != exploration_target_.identity()) return;
          std::optional<geometry_msgs::msg::Point> failure_position;
          if (odom_ && odom_->header.frame_id == message->header.frame_id)
          {
            const int64_t odom_ns = rclcpp::Time(odom_->header.stamp).nanoseconds();
            const auto& point = odom_->pose.pose.position;
            if (now_ns >= odom_ns && now_ns - odom_ns <= 5000000000LL &&
                std::isfinite(point.x) && std::isfinite(point.y))
              failure_position = point;
          }
          if (exploration_backoff_.reject(*message, now_ns, exploring, failure_position))
          {
            retire_exploration("vehicle_route_rejected");
            planner_.set_rejected_edges(exploration_backoff_.active_edges(now_ns));
            if (latest_graph_) planner_.update_graph(latest_graph_);
            RCLCPP_INFO(this->get_logger(), "Local vehicle planner rejected exploration route; selecting another frontier");
            this->plan_to_goal();
          }
        });
    observation_ack_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/observed_route_endpoint", 10,
        [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
          if (goal_pose_ || explicit_goal_received_ || !autonomous_exploration_enabled_) return;
          const auto identity = exploration_backoff_.identity(
              *message, this->get_clock()->now().nanoseconds());
          if (identity) exploration_target_.acknowledge(*identity);
        });
    route_status_pub_ = this->create_publisher<std_msgs::msg::String>("~/route_status", 10);
    retirement_pub_ = this->create_publisher<std_msgs::msg::String>("~/retired_exploration", 10);
    selection_ack_pub_ = this->create_publisher<std_msgs::msg::String>("~/selection_ack", 10);
    selection_sub_ = this->create_subscription<graphnav_msgs::msg::ExplorationChoice>(
        "~/vehicle_choice", 10, [this](const graphnav_msgs::msg::ExplorationChoice::ConstSharedPtr message) {
          accept_vehicle_choice(*message);
        });
    grid_map_debug_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("~/unexplored_space_map", 10);
    scores_debug_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/frontier_scores", 10);
  }

private:
  void clear_destination(bool reset_search, std_srvs::srv::Trigger::Response& response)
  {
    destination_fence_.clear(this->get_clock()->now().nanoseconds());
    goal_pose_.reset();
    explicit_goal_received_ = false;
    retire_exploration(reset_search ? "goal_reset" : "goal_release");
    if (reset_search) exploration_backoff_.clear();
    planner_.set_rejected_edges(exploration_backoff_.active_edges(
        this->get_clock()->now().nanoseconds()));
    if (latest_graph_) planner_.update_graph(latest_graph_);
    response.success = true;
    response.message = reset_search ? "Destination and search backoff reset."
                                    : "Destination released; search backoff retained.";
  }

  struct VehicleOption {
    std::string uuid;
    geometry_msgs::msg::Pose goal;
    double benefit;
  };

  bool known_vehicle_destination(const geometry_msgs::msg::Pose& goal)
  {
    // Planning mutates frontier/debug state. Probe a value copy so read-only
    // comparison cannot change the route owned by the active transaction.
    Planner probe = planner_;
    Eigen::Vector3d point(goal.position.x,goal.position.y,goal.position.z);
    const auto path = probe.plan_to_goal(point,goal_radius_,this->get_clock()->now());
    return !path.empty() && probe.route_complete();
  }

  void accept_vehicle_choice(const graphnav_msgs::msg::ExplorationChoice& choice)
  {
    const int64_t now = this->get_clock()->now().nanoseconds();
    const auto reply = [&](bool accepted, const std::string& reason) {
      std_msgs::msg::String ack;
      std::ostringstream s;
      s << "{\"stamp_ns\":" << now << ",\"source_stamp_ns\":"
        << rclcpp::Time(choice.header.stamp).nanoseconds()
        << ",\"exploration_id\":\"" << choice.exploration_id
        << "\",\"accepted\":" << (accepted ? "true" : "false")
        << ",\"reason\":\"" << reason << "\"}";
      ack.data = s.str(); selection_ack_pub_->publish(ack);
    };
    // Only identifiers we generated are echoed into JSON.
    if (!exploration_target_.active() || choice.exploration_id != exploration_target_.identity()) return;
    if (choice.header.stamp.sec <= 0 || choice.header.stamp.nanosec >= 1000000000u) return;
    geometry_msgs::msg::PoseStamped source;
    source.header = choice.header; source.pose.position = choice.route_endpoint;
    const auto owner = exploration_backoff_.identity(source, now);
    if (!owner || *owner != choice.exploration_id || goal_pose_ || explicit_goal_received_ ||
        !autonomous_exploration_enabled_) { reply(false, "inactive_owner"); return; }
    if (vehicle_cost_selected_) {
      reply(choice.chosen_uuid == exploration_target_.uuid(), "already_selected");
      plan_to_goal(); return;
    }
    if (!odom_ || odom_->header.frame_id != choice.header.frame_id ||
        now < rclcpp::Time(odom_->header.stamp).nanoseconds() ||
        now-rclcpp::Time(odom_->header.stamp).nanoseconds() > 500000000LL)
      { reply(false, "stale_origin"); return; }
    const auto& p = odom_->pose.pose;
    const auto& q = choice.evaluation_start.orientation;
    const auto heading = [](const auto& v) { return std::atan2(2*(v.w*v.z+v.x*v.y),1-2*(v.y*v.y+v.z*v.z)); };
    const double norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    const double moved = std::hypot(p.position.x-choice.evaluation_start.position.x,
                                  p.position.y-choice.evaluation_start.position.y);
    const double turned = std::remainder(heading(p.orientation)-heading(q), 2*std::acos(-1.));
    const auto& v = odom_->twist.twist.linear;
    if (!std::isfinite(norm) || std::abs(norm-1.) > .01 || !std::isfinite(moved) || moved > .25 ||
        !std::isfinite(turned) || std::abs(turned) > .1 || !std::isfinite(std::hypot(v.x,v.y)) ||
        std::hypot(v.x,v.y) > .15 || !std::isfinite(choice.vehicle_cost) || choice.vehicle_cost < 0.)
      { reply(false, "origin_changed"); return; }
    const auto found = std::find_if(vehicle_options_.begin(),vehicle_options_.end(),
        [&](const auto& option) { return option.uuid == choice.chosen_uuid; });
    if (found == vehicle_options_.end() || exploration_backoff_.postponed(
        found->goal.position,now,odom_->pose.pose.position) || !known_vehicle_destination(found->goal))
      { reply(false, "invalid_option"); return; }
    // Copy before retiring the old transaction (which clears its offers).
    const auto selected = *found;
    reply(true, "selected");
    if (selected.uuid != exploration_target_.uuid()) {
      retire_exploration("vehicle_cost_selection");
      geometry_msgs::msg::PoseStamped goal; goal.header = *latest_graph_header_;
      goal.pose = selected.goal;
      exploration_target_.begin(selected.uuid,goal,heading(selected.goal.orientation));
    }
    vehicle_cost_selected_ = true;
    vehicle_options_.clear();
    plan_to_goal();
  }

  void retire_exploration(const std::string& reason)
  {
    if (!exploration_target_.active()) return;
    std_msgs::msg::String message;
    message.data = "{\"stamp_ns\":" + std::to_string(this->get_clock()->now().nanoseconds()) +
        ",\"exploration_id\":\"" + exploration_target_.identity() +
        "\",\"reason\":\"" + reason + "\"}";
    retirement_pub_->publish(message);
    exploration_target_.clear();
    vehicle_options_.clear();
    vehicle_cost_selected_ = false;
  }

  std::optional<geometry_msgs::msg::PoseStamped> choose_exploration_goal()
  {
    if (!latest_graph_ || latest_graph_->current_node_idx >= latest_graph_->nodes.size())
      return std::nullopt;
    const auto class_it = std::find(latest_graph_->trav_classes.begin(),
                                    latest_graph_->trav_classes.end(), trav_class_);
    if (class_it == latest_graph_->trav_classes.end())
      return std::nullopt;
    const size_t class_idx = std::distance(latest_graph_->trav_classes.begin(), class_it);
    auto origin = latest_graph_->nodes[latest_graph_->current_node_idx].pose.position;
    std::optional<geometry_msgs::msg::Point> actual_origin;
    if (odom_ && odom_->header.frame_id == latest_graph_header_->frame_id)
    {
      const int64_t stamp = rclcpp::Time(odom_->header.stamp).nanoseconds();
      const int64_t now = this->get_clock()->now().nanoseconds();
      const auto& actual = odom_->pose.pose.position;
      if (stamp > 0 && now >= stamp && now-stamp <= 500000000LL &&
          std::isfinite(actual.x) && std::isfinite(actual.y)) { origin = actual; actual_origin = actual; }
    }
    if (exploration_target_.active())
    {
      const int64_t now = this->get_clock()->now().nanoseconds();
      if (exploration_target_.goal().header.frame_id != latest_graph_header_->frame_id)
      {
        retire_exploration("graph_frame_changed");
        exploration_backoff_.clear();
        exploration_visits_.clear();
      }
      else if (odom_ && exploration_target_.complete(*odom_, now, 1.0))
      {
        ++exploration_visits_[exploration_target_.uuid()];
        RCLCPP_INFO(this->get_logger(), "Exploration observed and retired: %s",
                    exploration_target_.identity().c_str());
        retire_exploration("observation_completed");
      }
      else if (held_frontier_resolved(class_idx, now))
        retire_exploration(exploration_target_.missing_frontier_expired() ?
            "frontier_missing" : "frontier_resolved");
      else if (exploration_backoff_.postponed(exploration_target_.goal().pose.position,
                                            now, actual_origin))
        retire_exploration("destination_backoff");
      else
        // Scoring updates can refine the route, but cannot steal its destination.
        // Retain the snapshot even if the mapper retires the frontier UUID.
        return exploration_target_.goal();
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
      bool heading_current = assessment == node.robot_traversability.end();
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
          heading_current = true;
          physical_confidence = assessment->confidence;
          physical_traversability = assessment->physical_traversability;
          if (physical_confidence >= 0.7 &&
              (!assessment->feasible || !std::isfinite(physical_traversability) ||
               physical_traversability <= 0.0 || !std::isfinite(assessment->dynamic_risk) ||
               assessment->dynamic_risk >= 1.0f)) continue;
        }
      }
      const auto& pos = node.pose.position;
      if (exploration_backoff_.postponed(pos, this->get_clock()->now().nanoseconds(),
                                        actual_origin)) continue;
      const double distance = std::hypot(pos.x - origin.x, pos.y - origin.y);
      if (distance <= goal_radius_)
        continue;
      double score = 0.3;
      for (const auto& property : node.properties)
        if (property.key == "frontier_scores" && !property.value.empty())
          score = *std::max_element(property.value.begin(), property.value.end());
      if (!std::isfinite(score) || score <= 1e-4)
        continue;
      if (heading_current && !node.heading_candidates.empty())
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
          false);
      if (!std::isfinite(utility)) continue;
      candidates.push_back({utility, i, uuid});
    }
    if (candidates.empty())
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "Exploration has no eligible destination; earliest postponed retry_ns=%ld",
          exploration_backoff_.next_retry(this->get_clock()->now().nanoseconds(),actual_origin));
      return std::nullopt;
    }
    std::stable_sort(candidates.begin(), candidates.end(),
        [](const auto& a, const auto& b) { return a.utility > b.utility; });
    const auto selected = candidates.begin();
    vehicle_options_.clear();
    vehicle_cost_selected_ = false;
    size_t examined = 0;
    for (const auto& candidate : candidates) {
      if (++examined > 8) break;
      const auto& node = latest_graph_->nodes[candidate.index];
      const auto heading = exploration_heading(node);
      const double distance = std::hypot(node.pose.position.x-origin.x,node.pose.position.y-origin.y);
      if (!heading || distance > 24. || !known_vehicle_destination(node.pose)) continue;
      auto pose = node.pose;
      pose.orientation.x = pose.orientation.y = 0.;
      pose.orientation.z = std::sin(*heading/2.); pose.orientation.w = std::cos(*heading/2.);
      vehicle_options_.push_back({candidate.uuid,pose,candidate.utility+.004*distance});
      if (vehicle_options_.size() == 4) break;
    }
    geometry_msgs::msg::PoseStamped goal;
    goal.header = *latest_graph_header_;
    goal.pose.position = latest_graph_->nodes[selected->index].pose.position;
    exploration_target_.begin(selected->uuid, goal,
        exploration_heading(latest_graph_->nodes[selected->index]));
    return exploration_target_.goal();
  }

  bool held_frontier_resolved(size_t class_idx, int64_t now)
  {
    std::optional<bool> frontier;
    for (const auto& node : latest_graph_->nodes)
      if (uuid_to_string(node.uuid) == exploration_target_.uuid())
      {
        // Presence with unclassified/moved geometry is not authoritative
        // absence. Conservatively retain it and break the missing sequence.
        frontier = true;
        const auto& p = node.pose.position;
        const auto& goal = exploration_target_.goal().pose.position;
        if (class_idx < node.trav_properties.size() &&
            std::hypot(p.x-goal.x,p.y-goal.y) <= .25)
          frontier = node.trav_properties[class_idx].is_frontier;
        break;
      }
    return exploration_target_.frontier_resolved(*latest_graph_header_, frontier,
        now, planner_.stale_observation_seconds_, missing_frontier_timeout_seconds_);
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
      // WildOS keeps the localized destination after the frontier prefix.
      // This is planning intent; the vehicle bridge still checks its rolling path.
      if (!exploring && !path.empty() && !planner_.route_complete())
        path.push_back(goal_vec);
      if (exploring && path.empty())
      {
        // Failed planning is not a completed observation. Temporarily back off
        // this destination using the same correlated rejection protocol.
        geometry_msgs::msg::PoseStamped refusal = *exploration_goal;
        refusal.header.stamp = this->get_clock()->now();
        exploration_backoff_.remember(refusal.header, refusal.pose.position,
            refusal.pose.position, exploration_target_.identity());
        exploration_backoff_.reject(refusal, this->get_clock()->now().nanoseconds(), true);
        retire_exploration("graph_path_unavailable");
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
      if (exploring)
      {
        encoded << ",\"exploration_id\":\"" << exploration_target_.identity() << "\""
                << ",\"endpoint_observed\":" << (exploration_target_.acknowledged() ? "true" : "false")
                << ",\"vehicle_cost_selected\":" << (vehicle_cost_selected_ ? "true" : "false");
        if (!vehicle_cost_selected_ && planner_.route_complete() && vehicle_options_.size() > 1) {
          encoded << ",\"vehicle_options\":[";
          for (size_t i=0; i<vehicle_options_.size(); ++i) {
            const auto& option=vehicle_options_[i]; const auto& q=option.goal.orientation;
            if (i) encoded << ",";
            encoded << "{\"uuid\":\"" << option.uuid << "\",\"goal\":["
              << option.goal.position.x << "," << option.goal.position.y << ","
              << std::atan2(2*q.w*q.z,1-2*q.z*q.z) << "],\"benefit\":" << option.benefit << "}";
          }
          encoded << "]";
        }
        if (exploration_target_.heading() && planner_.route_complete())
          encoded << ",\"observation_heading\":" << *exploration_target_.heading();
      }
      encoded << "}";
      status.data = encoded.str();
      route_status_pub_->publish(status);
      if (exploring && !path_msg.poses.empty())
      {
        std::vector<geometry_msgs::msg::Point> route_points;
        route_points.reserve(path_msg.poses.size());
        for (const auto& pose : path_msg.poses)
          route_points.push_back(pose.pose.position);
        exploration_backoff_.remember(path_msg.header, path_msg.poses.back().pose.position,
                                      goal_in_graph_frame.pose.position, exploration_target_.identity(),
                                      route_points);
      }
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
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr observation_ack_sub_;
  rclcpp::Subscription<graphnav_msgs::msg::ExplorationChoice>::SharedPtr selection_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr selection_ack_pub_;
  std::vector<VehicleOption> vehicle_options_;
  bool vehicle_cost_selected_ = false;
  ExplorationBackoff exploration_backoff_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr route_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr retirement_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_goal_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr release_goal_service_;
  DestinationFence destination_fence_;
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
  double missing_frontier_timeout_seconds_ = 10.;
  bool autonomous_exploration_enabled_ = false;
  bool explicit_goal_received_ = false;
  std::string trav_class_;
  ExplorationTarget exploration_target_;
  std::unordered_map<std::string, int> exploration_visits_;

  Planner planner_;
};

}  // namespace graphnav_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(graphnav_planner::PlannerNode)
