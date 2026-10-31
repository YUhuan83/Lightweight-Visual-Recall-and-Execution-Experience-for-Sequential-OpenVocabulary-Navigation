// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace wildos_ackermann_rpp {
inline double bounded_curvature(double requested, double radius) {
  if (!std::isfinite(requested) || !std::isfinite(radius) || radius <= 0.) {
    throw std::invalid_argument("Finite curvature and positive turning radius required");
  }
  return std::clamp(requested, -1. / radius, 1. / radius);
}
}
