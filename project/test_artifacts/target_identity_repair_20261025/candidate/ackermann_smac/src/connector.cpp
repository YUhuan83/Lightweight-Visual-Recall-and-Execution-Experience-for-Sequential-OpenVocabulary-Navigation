// SPDX-License-Identifier: Apache-2.0
#include "wildos_ackermann_smac/connector.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "ompl/base/spaces/ReedsSheppStateSpace.h"
#include "nav2_smac_planner/collision_checker.hpp"
#include "nav2_util/line_iterator.hpp"

namespace wildos_ackermann_smac {
bool safe(nav2_costmap_2d::Costmap2D & map,
  const nav2_costmap_2d::Footprint & footprint, Pose p, bool unknown)
{
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.yaw) ||
    footprint.size() < 3) { return false; }
  unsigned mx, my;
  if (!map.worldToMap(p.x, p.y, mx, my)) { return false; }
  auto blocked = [unknown](unsigned char cost) {
    return cost == 254 || (!unknown && cost == 255);
  };
  if (map.getCost(mx, my) == 253 || blocked(map.getCost(mx, my))) { return false; }
  const double c = std::cos(p.yaw), s = std::sin(p.yaw);
  std::vector<std::pair<double, double>> polygon;
  std::vector<std::pair<unsigned, unsigned>> corners;
  unsigned minx = map.getSizeInCellsX(), miny = map.getSizeInCellsY(), maxx = 0, maxy = 0;
  for (const auto & point : footprint) {
    double x = p.x + c * point.x - s * point.y, y = p.y + s * point.x + c * point.y;
    if (!std::isfinite(x) || !std::isfinite(y) || !map.worldToMap(x, y, mx, my)) {
      return false;
    }
    polygon.emplace_back(x, y); corners.emplace_back(mx, my);
    minx = std::min(minx, mx); miny = std::min(miny, my);
    maxx = std::max(maxx, mx); maxy = std::max(maxy, my);
  }
  for (unsigned i = 0; i < corners.size(); ++i) {
    const auto a = corners[i], b = corners[(i + 1) % corners.size()];
    for (nav2_util::LineIterator line(a.first, a.second, b.first, b.second);
      line.isValid(); line.advance()) {
      if (blocked(map.getCost(line.getX(), line.getY()))) { return false; }
    }
  }
  // Nav2 boundary raster plus known lethal interior. Unknown never hides a lethal cell.
  for (unsigned y = miny; y <= maxy; ++y) {
    for (unsigned x = minx; x <= maxx; ++x) {
      if (!blocked(map.getCost(x, y))) { continue; }
      double wx, wy; map.mapToWorld(x, y, wx, wy);
      bool inside = false;
      for (unsigned i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
        const auto a = polygon[i], b = polygon[j];
        if ((a.second > wy) != (b.second > wy) &&
          wx < (b.first - a.first) * (wy - a.second) / (b.second - a.second) + a.first) {
          inside = !inside;
        }
      }
      if (inside) { return false; }
    }
  }
  return true;
}

Curve connect(Pose from, Pose to, double radius, double min_section, double step,
  double maximum_length)
{
  if (!std::isfinite(radius) || radius <= 0 || !std::isfinite(step) || step <= 0 ||
    !std::isfinite(min_section) || min_section <= 0 ||
    (maximum_length != -1. && (!std::isfinite(maximum_length) || maximum_length <= 0.)) ||
    !std::isfinite(from.x) || !std::isfinite(from.y) || !std::isfinite(from.yaw) ||
    !std::isfinite(to.x) || !std::isfinite(to.y) || !std::isfinite(to.yaw)) { return {}; }
  ompl::base::ReedsSheppStateSpace space(radius);
  auto a = space.allocState(), b = space.allocState();
  auto sa = a->as<ompl::base::SE2StateSpace::StateType>();
  auto sb = b->as<ompl::base::SE2StateSpace::StateType>();
  sa->setXY(from.x, from.y); sa->setYaw(from.yaw);
  sb->setXY(to.x, to.y); sb->setYaw(to.yaw);
  const auto path = space.reedsShepp(a, b); space.freeState(a); space.freeState(b);
  Curve out; out.length = path.length() * radius;
  // Also limits allocation on unusually fine maps / malicious inputs.
  const double length_limit = maximum_length < 0. ? radius : maximum_length;
  if (!std::isfinite(out.length) || out.length > length_limit || out.length / step > 8192) {
    return {};
  }
  double section = 0; int previous = 0;
  for (unsigned i = 0; i < 5; ++i) {
    double length = path.length_[i] * radius;
    if (std::abs(length) < 1e-9) { continue; }
    int direction = length > 0 ? 1 : -1;
    if (previous && previous != direction) {
      if (section <= min_section) { return {}; }
      ++out.cusps; section = 0;
    }
    section += std::abs(length); previous = direction;
  }
  if (section <= min_section) { return {}; }
  out.poses.push_back(from); Pose current = from;
  for (unsigned i = 0; i < 5; ++i) {
    double length = path.length_[i] * radius;
    if (std::abs(length) < 1e-9) { continue; }
    double k = path.type_[i] == ompl::base::ReedsSheppStateSpace::RS_LEFT ? 1 / radius :
      path.type_[i] == ompl::base::ReedsSheppStateSpace::RS_RIGHT ? -1 / radius : 0;
    unsigned steps = static_cast<unsigned>(std::ceil(std::abs(length) / step));
    for (unsigned j = 1; j <= steps; ++j) {
      double travel = length * j / steps, angle = k * travel;
      double x = k == 0 ? travel : std::sin(angle) / k;
      double y = k == 0 ? 0 : (1 - std::cos(angle)) / k;
      out.poses.push_back({current.x + x * std::cos(current.yaw) - y * std::sin(current.yaw),
        current.y + x * std::sin(current.yaw) + y * std::cos(current.yaw), current.yaw + angle});
    }
    current = out.poses.back();
  }
  double yaw_error = std::atan2(std::sin(current.yaw - to.yaw), std::cos(current.yaw - to.yaw));
  if (std::hypot(current.x - to.x, current.y - to.y) > 1e-7 || std::abs(yaw_error) > 1e-7) {
    return {};
  }
  return out;
}

SearchResult search(nav2_costmap_2d::Costmap2D & map,
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
          ++out.evaluated;
          if (checker.inCollision(x, y, bin, lim.allow_unknown)) { continue; }
          // Match NodeHybrid float angle output, including wrap-around precision.
          float angle = bin * (2.f * static_cast<float>(M_PI) / lim.bins);
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
