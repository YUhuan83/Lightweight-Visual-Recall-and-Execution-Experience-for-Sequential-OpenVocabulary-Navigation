#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace graphnav_planner
{
// Keep unknown frontiers available for exploration, but prefer candidates on
// terrain that the mapper has measured to be physically traversable.
inline double exploration_candidate_utility(double heading_score,
                                            double distance_m,
                                            int visits,
                                            double physical_confidence,
                                            double physical_traversability,
                                            bool keep_current = false)
{
  if (!std::isfinite(heading_score) || !std::isfinite(distance_m) || distance_m < 0.0 || visits < 0)
    return -std::numeric_limits<double>::infinity();

  const double confidence = std::isfinite(physical_confidence) ?
      std::clamp(physical_confidence, 0.0, 1.0) : 0.0;
  const double traversability = std::isfinite(physical_traversability) ?
      std::clamp(physical_traversability, 0.0, 1.0) : 0.0;
  const double confidence_factor = 0.35 + 0.65 * confidence * traversability;
  const double adjusted_heading_score = heading_score > 0.0 ?
      heading_score * confidence_factor : heading_score;

  return adjusted_heading_score - 0.004 * distance_m - 0.08 * visits +
      (keep_current ? 0.15 : 0.0);
}
}  // namespace graphnav_planner
