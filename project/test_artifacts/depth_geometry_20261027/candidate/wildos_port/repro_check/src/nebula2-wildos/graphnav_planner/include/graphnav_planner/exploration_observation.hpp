#pragma once
#include <cmath>
#include <limits>
#include <optional>
#include <graphnav_msgs/msg/node.hpp>

namespace graphnav_planner
{
inline std::optional<double> exploration_heading(const graphnav_msgs::msg::Node& node)
{
  std::optional<double> result;
  double utility = -std::numeric_limits<double>::infinity();
  for (const auto& candidate : node.heading_candidates)
    if (candidate.traversable && std::isfinite(candidate.heading) &&
        std::isfinite(candidate.predicted_utility) && candidate.predicted_utility > utility)
    {
      result = candidate.heading;
      utility = candidate.predicted_utility;
    }
  return result;
}

// Acquisition time, rather than wall time, prevents pauses from completing a view.
class ExplorationObservation
{
  int64_t started_ = 0;
public:
  void clear() { started_ = 0; }
  bool complete(int64_t stamp, bool close, double yaw, double desired, double speed)
  {
    const double error = std::atan2(std::sin(yaw-desired), std::cos(yaw-desired));
    if (stamp <= 0 || !close || !std::isfinite(error) || !std::isfinite(speed) ||
        std::abs(error) > .35 || std::abs(speed) > .15)
    { clear(); return false; }
    if (started_ == 0 || stamp < started_) started_ = stamp;
    return stamp-started_ >= 1500000000LL;
  }
};
}
