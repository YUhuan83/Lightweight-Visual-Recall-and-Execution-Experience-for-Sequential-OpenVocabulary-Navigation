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
#include <optional>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>
#include "graphnav_planner/planner.hpp"

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

    graph_sub_ = this->create_subscription<graphnav_msgs::msg::NavigationGraph>(
        "~/nav_graph", 10, [this](const graphnav_msgs::msg::NavigationGraph::ConstSharedPtr msg) {
          this->planner_.update_graph(msg);
          this->latest_graph_ = msg;
          this->latest_graph_header_ = msg->header;
          this->plan_to_goal();
        });
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/goal_pose", 10, [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
          this->goal_pose_ = msg;
          this->explicit_goal_received_ = true;
          this->exploration_target_uuid_.clear();
          this->plan_to_goal();
        });
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "~/odom", 10, [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) { this->odom_ = msg; });

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("~/path", 10);
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
          ++exploration_visits_[exploration_target_uuid_];
          exploration_target_uuid_.clear();
        }
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
      if (assessment == node.robot_traversability.end() || !assessment->feasible ||
          assessment->physical_traversability <= 0.0f || assessment->dynamic_risk >= 1.0f)
        continue;
      const auto& pos = node.pose.position;
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
      const double utility = score - 0.08 * visits - 0.002 * distance +
          (uuid == exploration_target_uuid_ ? 0.15 : 0.0);
      candidates.push_back({utility, i, uuid});
    }
    if (candidates.empty())
      return std::nullopt;
    const auto selected = std::max_element(candidates.begin(), candidates.end(),
        [](const auto& a, const auto& b) { return a.utility < b.utility; });
    exploration_target_uuid_ = selected->uuid;
    geometry_msgs::msg::PoseStamped goal;
    goal.header = *latest_graph_header_;
    goal.pose.position = latest_graph_->nodes[selected->index].pose.position;
    goal.pose.orientation.w = 1.0;
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
      path_msg.poses.resize(path.size());
      for (size_t i = 0; i < path.size(); i++)
      {
        path_msg.poses[i].pose.position.x = path[i].x();
        path_msg.poses[i].pose.position.y = path[i].y();
        path_msg.poses[i].pose.position.z = path[i].z();
        if (i < path.size() - 1)
        {
          // heading towards next waypoint
          Eigen::Vector3d d = (path[i + 1] - path[i]).normalized();
          Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
          m.col(1) = Eigen::Vector3d::UnitZ().cross(d).normalized();
          m.col(0) = m.col(1).cross(m.col(2)).normalized();
          Eigen::Quaterniond q(m);
          path_msg.poses[i].pose.orientation.x = q.x();
          path_msg.poses[i].pose.orientation.y = q.y();
          path_msg.poses[i].pose.orientation.z = q.z();
          path_msg.poses[i].pose.orientation.w = q.w();
        }
        else  // last waypoint
        {
          path_msg.poses[i].pose.orientation = goal_in_graph_frame.pose.orientation;
        }
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
      if (!exploring && odom_)
      {
        try
        {
          geometry_msgs::msg::PoseStamped goal_in_odom_frame;
          goal_in_odom_frame = tf_buffer_.transform(goal, odom_->header.frame_id, tf2::durationFromSec(0.1));
          Eigen::Vector3d goal_vec(goal_in_odom_frame.pose.position.x, goal_in_odom_frame.pose.position.y,
                                   goal_in_odom_frame.pose.position.z);
          Eigen::Vector3d odom_vec(odom_->pose.pose.position.x, odom_->pose.pose.position.y,
                                   odom_->pose.pose.position.z);
          if ((goal_vec - odom_vec).norm() < goal_radius_)
          {
            goal_pose_.reset();  // clear goal
            explicit_goal_received_ = false;
          }
        }
        catch (const tf2::TransformException& ex)
        {
          RCLCPP_WARN(this->get_logger(), "Could not transform goal pose to odom frame: %s", ex.what());
        }
      }
    }
  }

  rclcpp::Subscription<graphnav_msgs::msg::NavigationGraph>::SharedPtr graph_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
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
  std::unordered_map<std::string, int> exploration_visits_;

  Planner planner_;
};

}  // namespace graphnav_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(graphnav_planner::PlannerNode)
