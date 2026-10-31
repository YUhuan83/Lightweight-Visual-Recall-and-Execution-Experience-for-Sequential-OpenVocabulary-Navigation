#pragma once
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

namespace graphnav_mapper {
struct TerrainMetrics {
  bool observed = false;
  double slope = 0., step = 0., roughness = 0.;
};
inline TerrainMetrics terrain_metrics(const std::vector<Eigen::Vector3d>& samples) {
  TerrainMetrics result;
  if (samples.size() < 5) return result;
  Eigen::MatrixXd design(samples.size(), 3);
  Eigen::VectorXd heights(samples.size());
  for (size_t i=0; i<samples.size(); ++i) {
    if (!samples[i].allFinite()) return result;
    design.row(i) << samples[i].x(), samples[i].y(), 1.;
    heights[i] = samples[i].z();
  }
  auto decomposition = design.colPivHouseholderQr();
  if (decomposition.rank() < 3) return result;
  const Eigen::Vector3d plane = decomposition.solve(heights);
  const Eigen::VectorXd residual = heights-design*plane;
  result.observed = true;
  result.slope = plane.head<2>().norm();  // Rise/run, consistent with robot_max_slope.
  result.step = residual.maxCoeff()-residual.minCoeff();
  result.roughness = std::sqrt(residual.squaredNorm()/samples.size());
  return result;
}
}
