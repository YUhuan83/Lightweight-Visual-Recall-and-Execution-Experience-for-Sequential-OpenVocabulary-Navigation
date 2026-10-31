// SPDX-License-Identifier: Apache-2.0
// The compute flow follows Nav2 RPP 1.1.20 (Humble), copyright 2020
// Shrijit Singh and Samsung Research America, under Apache License 2.0.
// All path transforms, regulation and footprint checking use the upstream
// implementation. Bound the arc before checking it and before publishing it.
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "wildos_ackermann_rpp/curvature.hpp"

namespace wildos_ackermann_rpp {
class Controller : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController {
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    RegulatedPurePursuitController::configure(parent, name, tf, costmap);
    auto node = parent.lock();
    nav2_util::declare_parameter_if_not_declared(node, name + ".minimum_turning_radius",
      rclcpp::ParameterValue(8.));
    node->get_parameter(name + ".minimum_turning_radius", radius_);
    bounded_curvature(0., radius_);
    if (use_rotate_to_heading_ || !allow_reversing_ || !use_collision_detection_) {
      throw nav2_core::PlannerException("Ackermann RPP requires collision checking, reversing and no in-place rotation");
    }
    // Preserve the rollout/controller contract through dynamic updates. A
    // preflight equality check alone does not protect a later handoff.
    std::vector<rclcpp::Parameter> frozen;
    for (const auto & field : {"minimum_turning_radius", "lookahead_dist",
      "use_velocity_scaled_lookahead_dist", "max_robot_pose_search_dist"}) {
      frozen.push_back(node->get_parameter(name + "." + field));
    }
    radius_guard_ = node->add_on_set_parameters_callback([frozen](const auto & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & p : parameters) {
        for (const auto & expected : frozen) {
          if (p.get_name() == expected.get_name() &&
            p.get_parameter_value() != expected.get_parameter_value()) {
            result.successful = false;
            result.reason = "Tracking geometry changes require a stopped controller reconfigure";
          }
        }
      }
      return result;
    });
  }

  void cleanup() override {
    radius_guard_.reset();
    RegulatedPurePursuitController::cleanup();
  }

  void setPlan(const nav_msgs::msg::Path & path) override {
    // GraphPathBridge submits one motion section per action. Preserve its
    // direction before upstream pruning removes the evidence of that section.
    int direction = 0;
    for (size_t i = 0; i < path.poses.size(); ++i) {
      const auto & p = path.poses[i].pose;
      const auto & q = p.orientation;
      if (!std::isfinite(p.position.x) || !std::isfinite(p.position.y) ||
          !std::isfinite(q.x) || !std::isfinite(q.y) ||
          !std::isfinite(q.z) || !std::isfinite(q.w) ||
          q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w < 1e-12) {
        throw nav2_core::PlannerException("Ackermann path contains an invalid pose");
      }
      if (i == 0) {continue;}
      const auto & previous = path.poses[i-1].pose;
      const double dx = p.position.x-previous.position.x;
      const double dy = p.position.y-previous.position.y;
      if (std::hypot(dx,dy) <= 1e-9) {continue;}
      const auto & h = previous.orientation;
      const double yaw = std::atan2(2.*(h.w*h.z+h.x*h.y),
        1.-2.*(h.y*h.y+h.z*h.z));
      const double longitudinal = dx*std::cos(yaw)+dy*std::sin(yaw);
      if (std::abs(longitudinal) <= 1e-9) {
        throw nav2_core::PlannerException("Ackermann path requires sideways motion");
      }
      const int edge_direction = longitudinal > 0. ? 1 : -1;
      if (direction != 0 && direction != edge_direction) {
        throw nav2_core::PlannerException("Ackermann path must be split at its reversing cusp");
      }
      direction = edge_direction;
    }
    if (path.poses.empty()) {
      throw nav2_core::PlannerException("Ackermann path is empty");
    }
    std::lock_guard<std::mutex> parameter_lock(mutex_);
    RegulatedPurePursuitController::setPlan(path);
    motion_direction_ = direction;
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & speed,
    nav2_core::GoalChecker * goal_checker) override {
    std::lock_guard<std::mutex> parameter_lock(mutex_);
    auto map = costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*map->getMutex());
    if (use_rotate_to_heading_ || !allow_reversing_ || !use_collision_detection_ || !goal_checker) {
      throw nav2_core::PlannerException("Ackermann controller runtime contract violated");
    }
    geometry_msgs::msg::Pose tolerance;
    geometry_msgs::msg::Twist velocity_tolerance;
    if (!goal_checker->getTolerances(tolerance, velocity_tolerance)) {
      throw nav2_core::PlannerException("Goal checker tolerances unavailable");
    }
    goal_dist_tol_ = tolerance.position.x;
    auto plan = transformGlobalPlan(pose);
    auto lookahead = getLookAheadDistance(speed);
    lookahead = std::min(lookahead, findVelocitySignChange(plan));
    auto carrot = getLookAheadPoint(lookahead, plan);
    carrot_pub_->publish(createCarrotMsg(carrot));
    const double x = carrot.pose.position.x, y = carrot.pose.position.y;
    const double squared_distance = x * x + y * y;
    const double requested = squared_distance > .001 ? 2. * y / squared_distance : 0.;
    const double applied = bounded_curvature(requested, radius_);
    double sign = x >= 0. ? 1. : -1.;
    double linear = desired_linear_vel_;
    // Keep upstream slowing for a sharp requested turn, cost and approach.
    const double pose_cost = costAtPose(pose.pose.position.x, pose.pose.position.y);
    applyConstraints(requested, speed, pose_cost,
      plan, linear, sign);
    double angular = linear * applied;
    // StoppedGoalChecker needs an explicit brake command: the upstream
    // approach regulator retains a nonzero minimum velocity at the endpoint.
    // Test the TRUE plan endpoint, not the costmap-cropped transformed plan.
    if (velocity_tolerance.linear.x >= 0. && velocity_tolerance.angular.z >= 0.) {
      auto target = global_plan_.poses.back();
      target.header.stamp = pose.header.stamp;
      geometry_msgs::msg::PoseStamped world_target;
      if (!transformPose(pose.header.frame_id, target, world_target)) {
        throw nav2_core::PlannerException("Cusp endpoint transform unavailable");
      }
      const auto heading = [](const auto & q) {
        return std::atan2(2. * (q.w*q.z + q.x*q.y), 1. - 2. * (q.y*q.y + q.z*q.z));
      };
      const double yaw_tolerance = std::abs(heading(tolerance.orientation));
      const double yaw_error = std::remainder(
        heading(world_target.pose.orientation) - heading(pose.pose.orientation), 2.*std::acos(-1.));
      if (std::hypot(world_target.pose.position.x-pose.pose.position.x,
                     world_target.pose.position.y-pose.pose.position.y) <= goal_dist_tol_ &&
          std::abs(yaw_error) < yaw_tolerance) {
        // The upstream footprint check scans the boundary. When stationary,
        // there is no future arc to sweep that boundary across the center cell.
        // Preserve its unknown-space policy, but never accept a lethal center.
        const bool allowed_unknown = pose_cost == nav2_costmap_2d::NO_INFORMATION &&
          costmap_ros_->getLayeredCostmap()->isTrackingUnknown();
        if (pose_cost >= nav2_costmap_2d::LETHAL_OBSTACLE && !allowed_unknown) {
          throw nav2_core::PlannerException("Ackermann cusp stop overlaps a lethal center cell");
        }
        linear = angular = 0.;
      }
    }
    if (linear != 0. && (motion_direction_ == 0 || linear*motion_direction_ < 0.)) {
      // A side-offset endpoint is not a new reversing cusp. Abort this action
      // for planning from the actual pose instead of cycling drive/reverse.
      // A genuinely reached stopped cusp has already produced zero above.
      throw nav2_core::PlannerException("Ackermann lookahead would reverse a single-direction path");
    }
    if (isCollisionImminent(pose, linear, angular, std::hypot(x, y))) {
      throw nav2_core::PlannerException("Ackermann RPP detected collision on its bounded command arc");
    }
    geometry_msgs::msg::TwistStamped command;
    command.header = pose.header;
    command.twist.linear.x = linear;
    command.twist.angular.z = angular;
    return command;
  }

private:
  double radius_ = 8.;
  int motion_direction_ = 0;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr radius_guard_;
};
}
PLUGINLIB_EXPORT_CLASS(wildos_ackermann_rpp::Controller, nav2_core::Controller)
