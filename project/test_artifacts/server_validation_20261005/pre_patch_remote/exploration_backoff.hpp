#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/time.hpp>

namespace graphnav_planner
{
constexpr double rejected_frontier_radius_m = 12.0;
constexpr int64_t rejected_frontier_backoff_ns = 120000000000LL;

// A local car planner's refusal is temporary exploration evidence. It does
// not mark a graph edge traversed or replace an explicit object destination.
class ExplorationBackoff
{
  struct Route {
    std_msgs::msg::Header header;
    geometry_msgs::msg::Point endpoint, destination;
    std::string uuid;
  };
  struct Region { geometry_msgs::msg::Point position; int64_t until; };
  std::deque<Route> routes_;
  std::vector<Region> regions_;
public:
  void clear() { routes_.clear(); regions_.clear(); }
  void remember(const std_msgs::msg::Header& header,
                const geometry_msgs::msg::Point& endpoint,
                const geometry_msgs::msg::Point& destination, const std::string& uuid)
  {
    routes_.push_back({header, endpoint, destination, uuid});
    if (routes_.size() > 128) routes_.pop_front();
  }
  bool reject(const geometry_msgs::msg::PoseStamped& message, int64_t now, bool exploring)
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
    regions_.erase(std::remove_if(regions_.begin(), regions_.end(),
        [now](const Region& region) { return region.until <= now; }), regions_.end());
    regions_.push_back({route->destination, now+rejected_frontier_backoff_ns});
    if (regions_.size() > 64) regions_.erase(regions_.begin());
    return true;
  }
  bool blocked(const geometry_msgs::msg::Point& p, int64_t now, double radius) const
  {
    return std::any_of(regions_.begin(), regions_.end(), [&](const Region& region) {
      return region.until > now && std::hypot(region.position.x-p.x, region.position.y-p.y) <= radius;
    });
  }
};
}
