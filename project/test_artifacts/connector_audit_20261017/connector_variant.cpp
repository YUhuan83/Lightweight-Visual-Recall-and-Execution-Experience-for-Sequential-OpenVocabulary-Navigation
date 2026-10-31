// SPDX-License-Identifier: Apache-2.0
#include "wildos_ackermann_smac/connector.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "ompl/base/spaces/ReedsSheppStateSpace.h"
#include "nav2_smac_planner/collision_checker.hpp"
#include "nav2_util/line_iterator.hpp"

namespace wildos_ackermann_smac {
SearchResult candidateSearch(nav2_costmap_2d::Costmap2D & map,
  const nav2_costmap_2d::Footprint & footprint, Pose from,
  const Limits & lim, Clock::time_point deadline)
{
  SearchResult out;
  if (!std::isfinite(lim.radius) || lim.radius <= 0 || lim.bins < 4 || lim.bins > 1024 ||
    lim.max_evaluations == 0 || lim.max_candidates == 0 || lim.max_candidates > 64 ||
    !safe(map, footprint, from, lim.allow_unknown)) { return out; }
  unsigned cx, cy; map.worldToMap(from.x, from.y, cx, cy);
  nav2_smac_planner::GridCollisionChecker checker(&map, lim.bins, nullptr);
  checker.setFootprint(footprint, false, 0);  // Force full body; do not depend on inflation here.
  const double bin_size = 2 * M_PI / lim.bins;
  const int nearest = static_cast<int>(std::round(from.yaw / bin_size));
  const int span = std::min(static_cast<int>(std::ceil(lim.radius / 2 / map.getResolution())),
    static_cast<int>(std::max(map.getSizeInCellsX(), map.getSizeInCellsY())));
  auto expired = [&]() {
    if (Clock::now() >= deadline) { out.timed_out = true; return true; }
    return out.evaluated >= lim.max_evaluations;
  };
  // Concentric rings and nearby headings first, bounded independently of map resolution.
  for (int ring = 0; ring <= span && !expired(); ++ring) {
    for (int dy = -ring; dy <= ring && !expired(); ++dy) {
      for (int dx = -ring; dx <= ring && !expired(); ++dx) {
        if (std::max(std::abs(dx), std::abs(dy)) != ring) { continue; }
        int x = static_cast<int>(cx) + dx, y = static_cast<int>(cy) + dy;
        if (x < 0 || y < 0 || x >= static_cast<int>(map.getSizeInCellsX()) ||
          y >= static_cast<int>(map.getSizeInCellsY())) { continue; }
        double wx, wy; map.mapToWorld(x, y, wx, wy);
        if (std::hypot(wx - from.x, wy - from.y) > lim.radius / 2) { continue; }
        for (unsigned rank = 0; rank < lim.bins && !expired(); ++rank) {
          int offset = rank == 0 ? 0 : (rank % 2 ? static_cast<int>((rank + 1) / 2) : -static_cast<int>(rank / 2));
          unsigned bin = (nearest + offset + 2 * static_cast<int>(lim.bins)) % lim.bins;
          // Any connector accepted by connect has length <= radius, hence
          // its net yaw change cannot exceed one radian at curvature <= 1/radius.
          float angle = bin * (2.f * static_cast<float>(M_PI) / lim.bins);
          if (std::abs(std::remainder(angle-from.yaw,2*M_PI)) > 1.+1e-6) { continue; }
          ++out.evaluated;
          if (checker.inCollision(x, y, bin, lim.allow_unknown)) { continue; }
          auto curve = connect(from, {wx, wy, angle}, lim.radius, lim.min_section,
            std::min(.05, map.getResolution() / 4));
          if (curve.poses.empty()) { continue; }
          bool valid = true;
          for (const auto & p : curve.poses) {
            if (Clock::now() >= deadline) { out.timed_out = true; valid = false; break; }
            if (!safe(map, footprint, p, lim.allow_unknown)) { valid = false; break; }
          }
          if (!valid) { continue; }
          out.curves.push_back(std::move(curve));
          std::stable_sort(out.curves.begin(), out.curves.end(), [](const Curve & a, const Curve & b) {
            return a.length + 3 * a.cusps < b.length + 3 * b.cusps;
          });
          if (out.curves.size() > lim.max_candidates) { out.curves.resize(lim.max_candidates); }
        }
      }
    }
    if (out.curves.size() >= lim.max_candidates) {
      // Finding enough curves at the first ring can retain costly turns and
      // miss a shorter reverse connection a few cells farther away. Distance
      // is a lower bound on any future curve's nonnegative length/cusp score.
      double centre_x, centre_y; map.mapToWorld(cx, cy, centre_x, centre_y);
      double next_min_distance = (ring + 1) * map.getResolution() -
        std::hypot(centre_x - from.x, centre_y - from.y);
      const auto & worst = out.curves.back();
      if (next_min_distance > worst.length + 3 * worst.cusps) { break; }
    }
  }
  return out;
}
}  // namespace wildos_ackermann_smac
