#pragma once
#include <cmath>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/header.hpp>
#include "graphnav_planner/exploration_observation.hpp"

namespace graphnav_planner
{
// One exploration transaction spans graph updates and all vehicle maneuvers.
// A missing frontier in a newer snapshot does not revoke an accepted destination.
class ExplorationTarget
{
  uint64_t serial_ = 0;
  const std::string instance_ = std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count());
  std::optional<geometry_msgs::msg::PoseStamped> goal_;
  std::string uuid_, identity_;
  std::optional<double> heading_;
  bool endpoint_observed_ = false;
  int64_t frontier_source_ns_ = 0;
  ExplorationObservation observation_;
public:
  bool active() const { return goal_.has_value(); }
  const auto& goal() const { return *goal_; }
  const auto& uuid() const { return uuid_; }
  const auto& identity() const { return identity_; }
  const auto& heading() const { return heading_; }
  bool acknowledged() const { return endpoint_observed_; }
  void clear()
  {
    goal_.reset(); uuid_.clear(); identity_.clear(); heading_.reset();
    endpoint_observed_ = false; observation_.clear(); frontier_source_ns_ = 0;
  }
  bool begin(const std::string& uuid, geometry_msgs::msg::PoseStamped goal,
             std::optional<double> heading)
  {
    if (active() || uuid.empty() || goal.header.frame_id.empty() ||
        !std::isfinite(goal.pose.position.x) || !std::isfinite(goal.pose.position.y) ||
        (heading && !std::isfinite(*heading))) return false;
    uuid_ = uuid; identity_ = uuid + ":" + instance_ + ":" + std::to_string(++serial_);
    heading_ = heading;
    goal.pose.orientation.x = goal.pose.orientation.y = 0.;
    goal.pose.orientation.z = std::sin(heading.value_or(0.) / 2.);
    goal.pose.orientation.w = std::cos(heading.value_or(0.) / 2.);
    goal_ = std::move(goal); endpoint_observed_ = false; observation_.clear();
    frontier_source_ns_ = 0;
    return true;
  }
  bool frontier_resolved(const std_msgs::msg::Header& source,
                         std::optional<bool> is_frontier, int64_t now,
                         double maximum_age_seconds)
  {
    // An authoritative non-frontier update ends an exploration intention,
    // not an object search and not a successful observation. Missing nodes
    // and score changes alone do not revoke ownership.
    if (!active() || source.frame_id != goal_->header.frame_id ||
        source.stamp.sec < 0 || source.stamp.nanosec >= 1000000000u ||
        !std::isfinite(maximum_age_seconds) || maximum_age_seconds <= 0.) return false;
    const int64_t stamp = rclcpp::Time(source.stamp).nanoseconds();
    const int64_t selected = rclcpp::Time(goal_->header.stamp).nanoseconds();
    if (stamp <= 0 || stamp <= selected || stamp <= frontier_source_ns_ || now < stamp ||
        static_cast<double>(now-stamp)*1e-9 > maximum_age_seconds) return false;
    frontier_source_ns_ = stamp;
    return is_frontier.has_value() && !*is_frontier;
  }
  bool acknowledge(const std::string& identity)
  {
    if (!active() || identity != identity_) return false;
    endpoint_observed_ = true;
    return true;
  }
  bool complete(const nav_msgs::msg::Odometry& odom, int64_t now, double radius)
  {
    const auto& p = odom.pose.pose.position;
    const auto& q = odom.pose.pose.orientation;
    const auto& v = odom.twist.twist.linear;
    const int64_t stamp = rclcpp::Time(odom.header.stamp).nanoseconds();
    const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (!active() || !endpoint_observed_ || odom.header.frame_id != goal_->header.frame_id ||
        stamp <= 0 || now < stamp || now-stamp > 500000000LL ||
        !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(norm) ||
        std::abs(norm-1.) > .01 || !std::isfinite(radius) || radius <= 0.)
    { observation_.clear(); return false; }
    const double yaw = std::atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z));
    return observation_.complete(stamp,
        std::hypot(p.x-goal_->pose.position.x, p.y-goal_->pose.position.y) <= radius,
        yaw, heading_.value_or(yaw), std::hypot(v.x, v.y));
  }
};
}
