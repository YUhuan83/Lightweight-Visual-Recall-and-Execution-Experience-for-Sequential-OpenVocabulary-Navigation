// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <chrono>
#include <vector>
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"

namespace wildos_ackermann_smac {
using Clock = std::chrono::steady_clock;
struct Pose { double x, y, yaw; };
struct Curve { std::vector<Pose> poses; double length = 0; unsigned cusps = 0; };
struct SearchResult { std::vector<Curve> curves; unsigned evaluated = 0; bool timed_out = false; };
struct Limits {
  double radius = 8., min_section = .4;
  unsigned bins = 72, max_evaluations = 6000, max_candidates = 6;
  bool allow_unknown = true;
};
bool safe(nav2_costmap_2d::Costmap2D & map,
  const nav2_costmap_2d::Footprint & footprint, Pose pose, bool allow_unknown);
Curve connect(Pose from, Pose to, double radius, double min_section, double sample_step);
// Caller owns the map lock. Deadline checked during curve collision sampling too.
SearchResult search(nav2_costmap_2d::Costmap2D & map,
  const nav2_costmap_2d::Footprint & footprint, Pose from,
  const Limits & limits, Clock::time_point deadline);
}  // namespace wildos_ackermann_smac
