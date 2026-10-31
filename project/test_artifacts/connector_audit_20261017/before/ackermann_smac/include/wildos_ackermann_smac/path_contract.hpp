// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "wildos_ackermann_smac/connector.hpp"
namespace wildos_ackermann_smac {
// Same directional-window contract as GraphPathBridge's geometry admission.
// This checks geometry only; current-map footprint checks remain mandatory.
bool curvatureFeasible(const std::vector<Pose> & path, double resolution,
  double radius, unsigned bins);
}
