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
constexpr double rejected_frontier_radius_m = 12.0;
constexpr double rejected_stall_region_radius_m = 8.0;
constexpr int64_t rejected_frontier_backoff_ns = 120000000000LL;
constexpr double rejected_edge_match_radius_m = 6.0;

using RejectedGraphEdge = std::pair<Eigen::Vector2d, Eigen::Vector2d>;

// A local car planner's refusal is temporary exploration evidence. It does
// not mark a graph edge traversed or replace an explicit object destination.
class ExplorationBackoff
{
  struct Route {
    std_msgs::msg::Header header;
    geometry_msgs::msg::Point endpoint, destination;
    std::string uuid;
    std::vector<geometry_msgs::msg::Point> path;
  };
  struct Region { geometry_msgs::msg::Point position; int64_t until; double radius; };
  struct RejectedEdge { RejectedGraphEdge edge; int64_t until; };
  std::deque<Route> routes_;
  std::vector<Region> regions_;
  std::vector<RejectedEdge> rejected_edges_;
public:
  void clear() { routes_.clear(); regions_.clear(); rejected_edges_.clear(); }
  void remember(const std_msgs::msg::Header& header,
                const geometry_msgs::msg::Point& endpoint,
                const geometry_msgs::msg::Point& destination, const std::string& uuid,
                const std::vector<geometry_msgs::msg::Point>& path = {})
  {
    routes_.push_back({header, endpoint, destination, uuid, path});
    if (routes_.size() > 128) routes_.pop_front();
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
    regions_.erase(std::remove_if(regions_.begin(), regions_.end(),
        [now](const Region& region) { return region.until <= now; }), regions_.end());
    regions_.push_back({route->destination, now+rejected_frontier_backoff_ns,
                        rejected_frontier_radius_m});
    if (regions_.size() > 64) regions_.erase(regions_.begin());

    // The route endpoint may be far beyond the actual obstruction. When the
    // latest odometry lies close to the issued graph path, temporarily remove
    // the graph edge at the failure location so another frontier cannot send
    // the vehicle through the same local bottleneck.
    if (failure_position && std::isfinite(failure_position->x) &&
        std::isfinite(failure_position->y) && route->path.size() >= 2)
    {
      double closest_distance_sq = std::numeric_limits<double>::infinity();
      size_t closest_segment = route->path.size();
      for (size_t i = 0; i + 1 < route->path.size(); ++i)
      {
        const auto& a = route->path[i];
        const auto& b = route->path[i + 1];
        const double dx = b.x - a.x, dy = b.y - a.y;
        const double length_sq = dx * dx + dy * dy;
        if (!std::isfinite(length_sq) || length_sq <= 1e-8) continue;
        const double t = std::clamp(((failure_position->x - a.x) * dx +
                                     (failure_position->y - a.y) * dy) / length_sq,
                                    0.0, 1.0);
        const double ex = failure_position->x - (a.x + t * dx);
        const double ey = failure_position->y - (a.y + t * dy);
        const double distance_sq = ex * ex + ey * ey;
        if (distance_sq < closest_distance_sq)
        {
          closest_distance_sq = distance_sq;
          closest_segment = i;
        }
      }
      if (closest_segment + 1 < route->path.size() &&
          closest_distance_sq <= rejected_edge_match_radius_m * rejected_edge_match_radius_m)
      {
        // The endpoint can be far beyond the obstruction. Temporarily avoid
        // nearby frontiers around the measured stall pose as well, so the
        // next candidate cannot repeatedly enter the same local bottleneck.
        regions_.push_back({*failure_position, now+rejected_frontier_backoff_ns,
                            rejected_stall_region_radius_m});
        if (regions_.size() > 64) regions_.erase(regions_.begin());
        const auto& a = route->path[closest_segment];
        const auto& b = route->path[closest_segment + 1];
        RejectedGraphEdge edge{
          Eigen::Vector2d(a.x, a.y), Eigen::Vector2d(b.x, b.y)};
        const auto same_edge = [&edge](const RejectedEdge& row) {
          constexpr double tolerance = 0.25;
          return ((row.edge.first - edge.first).norm() <= tolerance &&
                  (row.edge.second - edge.second).norm() <= tolerance) ||
                 ((row.edge.first - edge.second).norm() <= tolerance &&
                  (row.edge.second - edge.first).norm() <= tolerance);
        };
        rejected_edges_.erase(std::remove_if(rejected_edges_.begin(), rejected_edges_.end(),
            [now](const RejectedEdge& row) { return row.until <= now; }), rejected_edges_.end());
        const auto existing = std::find_if(rejected_edges_.begin(), rejected_edges_.end(), same_edge);
        if (existing != rejected_edges_.end())
          existing->until = now + rejected_frontier_backoff_ns;
        else
          rejected_edges_.push_back({edge, now + rejected_frontier_backoff_ns});
        if (rejected_edges_.size() > 64) rejected_edges_.erase(rejected_edges_.begin());
      }
    }
    return true;
  }
  std::vector<RejectedGraphEdge> active_edges(int64_t now) const
  {
    std::vector<RejectedGraphEdge> active;
    for (const auto& row : rejected_edges_)
      if (row.until > now) active.push_back(row.edge);
    return active;
  }
  bool blocked(const geometry_msgs::msg::Point& p, int64_t now, double radius) const
  {
    return std::any_of(regions_.begin(), regions_.end(), [&](const Region& region) {
      return region.until > now && std::hypot(region.position.x-p.x, region.position.y-p.y) <=
          std::max(radius, region.radius);
    });
  }
};
}
