#pragma once
#include <cstdint>

namespace graphnav_planner
{
// GoalPose and Trigger use separate DDS channels. A completed clear does not
// drain previously published goals; accept only newly stamped destinations.
class DestinationFence
{
  int64_t cleared_at_ = 0;
  int64_t accepted_at_ = 0;
public:
  void clear(int64_t now)
  {
    cleared_at_ = now;
    accepted_at_ = 0;
  }
  bool accept(int64_t stamp, int64_t now)
  {
    if (now <= 0 || stamp <= cleared_at_ || stamp <= accepted_at_ || stamp > now)
      return false;
    accepted_at_ = stamp;
    return true;
  }
};
}
