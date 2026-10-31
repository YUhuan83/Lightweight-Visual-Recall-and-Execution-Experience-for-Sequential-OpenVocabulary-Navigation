#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <string>
#include <vector>
#include <optional>
#include <utility>
#include <Eigen/Core>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/time.hpp>

namespace graphnav_planner
{
constexpr int64_t rejected_frontier_backoff_ns = 120000000000LL;
constexpr double rejected_destination_tolerance_m = 0.25;
constexpr double rejected_start_tolerance_m = 0.8;

using RejectedGraphEdge = std::pair<Eigen::Vector2d, Eigen::Vector2d>;

// A refusal postpones one destination from the measured failure position.
// It contains no obstacle geometry and must not invalidate an area or edge.
class ExplorationBackoff
{
  struct Route {
    std_msgs::msg::Header header;
    geometry_msgs::msg::Point endpoint, destination;
    std::string uuid;
    std::vector<geometry_msgs::msg::Point> path;
  };
  struct Attempt {
    geometry_msgs::msg::Point destination;
    std::optional<geometry_msgs::msg::Point> start;
    int64_t until;
  };
  std::deque<Route> routes_;
  std::vector<Attempt> attempts_;
public:
  void clear() { routes_.clear(); attempts_.clear(); }
  void remember(const std_msgs::msg::Header& header,
                const geometry_msgs::msg::Point& endpoint,
                const geometry_msgs::msg::Point& destination, const std::string& uuid,
                const std::vector<geometry_msgs::msg::Point>& path = {})
  {
    routes_.push_back({header, endpoint, destination, uuid, path});
    if (routes_.size() > 128) routes_.pop_front();
  }
  std::optional<std::string> identity(const geometry_msgs::msg::PoseStamped& message,
                                    int64_t now) const
  {
    if (message.header.stamp.sec < 0 || message.header.stamp.nanosec >= 1000000000u)
      return std::nullopt;
    const int64_t stamp = rclcpp::Time(message.header.stamp).nanoseconds();
    if (stamp <= 0 || now < stamp || now-stamp > 60000000000LL ||
        !std::isfinite(message.pose.position.x) || !std::isfinite(message.pose.position.y))
      return std::nullopt;
    const auto row = std::find_if(routes_.rbegin(), routes_.rend(), [&](const Route& route) {
      return route.header.frame_id == message.header.frame_id &&
          rclcpp::Time(route.header.stamp).nanoseconds() == stamp &&
          std::hypot(route.endpoint.x-message.pose.position.x,
                     route.endpoint.y-message.pose.position.y) < 1e-5;
    });
    return row == routes_.rend() ? std::nullopt : std::optional<std::string>(row->uuid);
  }
  bool reject(const geometry_msgs::msg::PoseStamped& message, int64_t now, bool exploring,
              const std::optional<geometry_msgs::msg::Point>& failure_position = std::nullopt)
  {
    if (!exploring || message.header.stamp.sec < 0 || message.header.stamp.nanosec >= 1000000000u)
      return false;
    const int64_t stamp = rclcpp::Time(message.header.stamp).nanoseconds();
    if (stamp <= 0 || now < stamp || now-stamp > 60000000000LL) return false;
    const auto& p = message.pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) return false;
    const auto route = std::find_if(routes_.rbegin(), routes_.rend(), [&](const Route& row) {
      return row.header.frame_id == message.header.frame_id &&
          rclcpp::Time(row.header.stamp).nanoseconds() == stamp &&
          std::hypot(row.endpoint.x-p.x, row.endpoint.y-p.y) < 1e-5;
    });
    if (route == routes_.rend()) return false;
    std::optional<geometry_msgs::msg::Point> start;
    if (failure_position && std::isfinite(failure_position->x) && std::isfinite(failure_position->y))
      start = failure_position;
    attempts_.erase(std::remove_if(attempts_.begin(), attempts_.end(),
        [now](const Attempt& a) { return a.until <= now; }), attempts_.end());
    attempts_.push_back({route->destination, start, now+rejected_frontier_backoff_ns});
    if (attempts_.size() > 64) attempts_.erase(attempts_.begin());
    return true;
  }
  std::vector<RejectedGraphEdge> active_edges(int64_t /*now*/) const
  {
    return {};  // Legacy callers cannot infer blocked edges from a refusal.
  }
  bool same_start(const Attempt& a,
      const std::optional<geometry_msgs::msg::Point>& current) const
  {
    return !a.start || !current || !std::isfinite(current->x) || !std::isfinite(current->y) ||
        std::hypot(a.start->x-current->x,a.start->y-current->y) <= rejected_start_tolerance_m;
  }
  bool postponed(const geometry_msgs::msg::Point& p, int64_t now,
      const std::optional<geometry_msgs::msg::Point>& current = std::nullopt) const
  {
    return std::any_of(attempts_.begin(), attempts_.end(), [&](const Attempt& a) {
      return a.until > now && same_start(a,current) &&
          std::hypot(a.destination.x-p.x,a.destination.y-p.y) <= rejected_destination_tolerance_m;
    });
  }
  bool blocked(const geometry_msgs::msg::Point& p, int64_t now, double /*legacy_radius*/) const
  { return postponed(p,now); }
  int64_t next_retry(int64_t now,
      const std::optional<geometry_msgs::msg::Point>& current = std::nullopt) const
  {
    int64_t retry = 0;
    for (const auto& a : attempts_)
      if (a.until > now && same_start(a,current) && (retry == 0 || a.until < retry)) retry = a.until;
    return retry;
  }
};
}
