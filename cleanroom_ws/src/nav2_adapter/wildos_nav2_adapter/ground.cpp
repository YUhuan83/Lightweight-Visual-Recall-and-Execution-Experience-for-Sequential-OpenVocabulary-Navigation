// SPDX-License-Identifier: Apache-2.0
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include <Eigen/SVD>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <tuple>
#include <vector>

namespace py = pybind11;
using Matrix = Eigen::MatrixXd;
using Vector = Eigen::VectorXd;
using Plane = std::array<double, 3>;
using CellKey = std::pair<std::int64_t, std::int64_t>;

struct Return {
  double x;
  double y;
  double z;
  std::size_t original;
};

struct Support {
  double x;
  double y;
  double z;
  std::size_t count;
  CellKey key;
};

struct Patch {
  std::vector<std::size_t> support;
  std::vector<std::size_t> matched;
  Plane plane;
  double x;
  double y;
};

double quantile(std::vector<double> values, double fraction)
{
  std::sort(values.begin(), values.end());
  const double position = fraction * static_cast<double>(values.size() - 1);
  const auto index = static_cast<std::size_t>(position);
  const double weight = position - static_cast<double>(index);
  return values[index] + (values[std::min(index + 1, values.size() - 1)] - values[index]) * weight;
}

Eigen::Index rank(const Matrix & matrix)
{
  Eigen::JacobiSVD<Matrix> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const auto singular = svd.singularValues();
  if (singular.size() == 0) {
    return 0;
  }
  const double cutoff = singular[0] * static_cast<double>(std::max(matrix.rows(), matrix.cols())) *
    std::numeric_limits<double>::epsilon();
  return (singular.array() > cutoff).count();
}

bool fitted(const std::vector<std::size_t> & indices, const std::vector<Support> & supports,
  const std::vector<std::size_t> & counts, double obstacle_height, double tolerance, Plane & output,
  std::size_t minimum_cells = 12)
{
  if (indices.size() < minimum_cells) {
    return false;
  }
  std::size_t return_count = 0;
  for (const auto index : indices) {
    return_count += counts[index];
  }
  if (return_count < 100) {
    return false;
  }

  const Eigen::Index rows = static_cast<Eigen::Index>(indices.size());
  Matrix matrix(rows, 3);
  Vector heights(rows);
  for (Eigen::Index row = 0; row < rows; ++row) {
    const auto & point = supports[indices[static_cast<std::size_t>(row)]];
    matrix(row, 0) = point.x;
    matrix(row, 1) = point.y;
    matrix(row, 2) = 1.0;
    heights[row] = point.z;
  }

  std::vector<bool> keep(indices.size(), true);
  Vector error(rows);
  Eigen::Vector3d value = Eigen::Vector3d::Zero();
  for (int iteration = 0; iteration < 4; ++iteration) {
    const auto kept = static_cast<Eigen::Index>(std::count(keep.begin(), keep.end(), true));
    if (kept < static_cast<Eigen::Index>(minimum_cells)) {
      return false;
    }
    Matrix selected(kept, 3);
    Vector selected_heights(kept);
    Eigen::Index destination = 0;
    for (Eigen::Index row = 0; row < rows; ++row) {
      if (keep[static_cast<std::size_t>(row)]) {
        selected.row(destination) = matrix.row(row);
        selected_heights[destination] = heights[row];
        ++destination;
      }
    }
    Eigen::JacobiSVD<Matrix> svd(selected, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const auto singular = svd.singularValues();
    const double cutoff = singular[0] * static_cast<double>(std::max(selected.rows(), selected.cols())) *
      std::numeric_limits<double>::epsilon();
    if ((singular.array() > cutoff).count() < 3) {
      return false;
    }
    const Matrix basis = svd.matrixV() * singular.cwiseInverse().asDiagonal();
    value = basis * (svd.matrixU().transpose() * selected_heights);
    const Matrix leverage_basis = matrix * basis;
    std::vector<double> errors(indices.size());
    std::vector<double> ordered_errors;
    ordered_errors.reserve(indices.size());
    std::vector<double> leverages(indices.size());
    for (Eigen::Index row = 0; row < rows; ++row) {
      error[row] = heights[row] - matrix.row(row).dot(value);
      errors[static_cast<std::size_t>(row)] = error[row];
      ordered_errors.push_back(error[row]);
      leverages[static_cast<std::size_t>(row)] = leverage_basis.row(row).squaredNorm();
    }
    const double lower = quantile(ordered_errors, .05) - .05;
    const double upper = quantile(ordered_errors, .65) + .05;
    std::vector<bool> updated(indices.size());
    for (std::size_t row = 0; row < indices.size(); ++row) {
      const double leverage = leverages[row];
      const double independent_error = std::abs(errors[row]) / std::max(1.0 - leverage, 1e-8);
      updated[row] = errors[row] >= lower && errors[row] <= upper &&
        leverage < 1.0 - 1e-8 && independent_error < obstacle_height;
    }
    if (updated == keep) {
      break;
    }
    keep = std::move(updated);
  }
  if (std::count(keep.begin(), keep.end(), true) < static_cast<std::ptrdiff_t>(minimum_cells) ||
    std::hypot(value[0], value[1]) > .6) {
    return false;
  }
  std::vector<double> absolute_errors;
  absolute_errors.reserve(indices.size());
  for (std::size_t i = 0; i < indices.size(); ++i) {
    absolute_errors.push_back(std::abs(error[static_cast<Eigen::Index>(i)]));
  }
  if (quantile(absolute_errors, .9) > tolerance) {
    return false;
  }
  output = {value[0], value[1], value[2]};
  return true;
}

py::tuple ground_patch_model(
  py::array_t<double, py::array::c_style | py::array::forcecast> point_array,
  py::array_t<double, py::array::c_style | py::array::forcecast> plane_array,
  double radius, double obstacle_height, double max_slope)
{
  const auto points = point_array.request();
  const auto plane_info = plane_array.request();
  if (points.ndim != 2 || points.shape[1] != 3 || plane_info.ndim != 1 || plane_info.shape[0] != 3) {
    throw py::value_error("points must be Nx3 and plane must contain three values");
  }
  const auto count = static_cast<std::size_t>(points.shape[0]);
  py::array_t<bool> result(count);
  py::array_t<double> heights(count);
  auto * output = static_cast<bool *>(result.request().ptr);
  auto * height = static_cast<double *>(heights.request().ptr);
  std::fill(output, output + count, false);
  std::fill(height, height + count, std::numeric_limits<double>::quiet_NaN());
  if (count == 0) {
    return py::make_tuple(result, heights);
  }
  const auto * xyz = static_cast<const double *>(points.ptr);
  const auto * plane_values = static_cast<const double *>(plane_info.ptr);
  const Plane plane{plane_values[0], plane_values[1], plane_values[2]};
  const double tolerance = std::min(.05, obstacle_height / 2.0);

  std::vector<Return> eligible;
  eligible.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const double z = xyz[i * 3 + 2];
    if (z >= -2.0 && z <= 1.2) {
      eligible.push_back({xyz[i * 3], xyz[i * 3 + 1], z, i});
    }
  }
  if (eligible.empty()) {
    return py::make_tuple(result, heights);
  }

  std::map<CellKey, std::vector<Return>> returns_by_cell;
  for (const auto & point : eligible) {
    const auto cx = static_cast<std::int64_t>(std::floor(point.x / .5));
    const auto cy = static_cast<std::int64_t>(std::floor(point.y / .5));
    returns_by_cell[{cx, cy}].push_back(point);
  }
  std::vector<Support> supports;
  std::vector<std::size_t> return_counts;
  std::map<CellKey, std::size_t> cell_indices;
  supports.reserve(returns_by_cell.size());
  return_counts.reserve(returns_by_cell.size());
  for (auto & entry : returns_by_cell) {
    auto & cell_returns = entry.second;
    const auto selected = std::min_element(cell_returns.begin(), cell_returns.end(),
      [](const Return & lhs, const Return & rhs) {
        return std::tie(lhs.z, lhs.x, lhs.y) < std::tie(rhs.z, rhs.x, rhs.y);
      });
    const auto index = supports.size();
    cell_indices.emplace(entry.first, index);
    supports.push_back({selected->x, selected->y, selected->z, cell_returns.size(), entry.first});
    return_counts.push_back(cell_returns.size());
  }

  std::vector<double> residual(supports.size());
  std::vector<bool> known(supports.size(), false);
  std::vector<std::size_t> seed_indices;
  for (std::size_t i = 0; i < supports.size(); ++i) {
    const auto & point = supports[i];
    residual[i] = point.z - (point.x * plane[0] + point.y * plane[1] + plane[2]);
    if (std::hypot(point.x, point.y) <= std::min(4.0, radius) &&
      std::abs(residual[i]) < tolerance) {
      known[i] = true;
      seed_indices.push_back(i);
    }
  }
  if (seed_indices.empty()) {
    return py::make_tuple(result, heights);
  }

  std::set<CellKey> tile_set;
  for (const auto & point : supports) {
    tile_set.emplace(static_cast<std::int64_t>(std::floor(point.x / 2.0)),
      static_cast<std::int64_t>(std::floor(point.y / 2.0)));
  }
  std::vector<Patch> patches;
  std::vector<std::vector<std::size_t>> members(supports.size());
  for (const auto & tile : tile_set) {
    const double center_x = (static_cast<double>(tile.first) + .5) * 2.0;
    const double center_y = (static_cast<double>(tile.second) + .5) * 2.0;
    std::vector<std::size_t> indices;
    for (std::size_t i = 0; i < supports.size(); ++i) {
      if (std::hypot(supports[i].x - center_x, supports[i].y - center_y) <= 4.0) {
        indices.push_back(i);
      }
    }
    Plane local{};
    if (!fitted(indices, supports, return_counts, obstacle_height, tolerance, local)) {
      continue;
    }
    const auto patch_id = patches.size();
    Patch patch;
    patch.support = indices;
    patch.plane = local;
    patch.x = center_x;
    patch.y = center_y;
    for (const auto index : indices) {
      const auto & point = supports[index];
      const double error = std::abs(point.z -
        (point.x * local[0] + point.y * local[1] + local[2]));
      if (error < tolerance) {
        patch.matched.push_back(index);
      }
    }
    patches.push_back(std::move(patch));
    for (const auto index : patches.back().matched) {
      members[index].push_back(patch_id);
    }
  }

  std::vector<bool> activated(patches.size(), false);
  // Overlap is a boundary of an already fitted patch, not another full patch.
  // Six independent cells overdetermine its three plane coefficients. Full
  // patches still require twelve cells; both fits retain 100 returns, rank,
  // leverage, residual, height-continuity and slope checks.
  constexpr std::size_t shared_plane_support = 6;
  std::vector<std::size_t> observed(patches.size(), 0);
  std::vector<Plane> coefficients(supports.size(), plane);
  std::vector<Plane> references(supports.size(), plane);
  std::vector<double> reference_distance(supports.size(), std::numeric_limits<double>::infinity());
  std::vector<double> best(supports.size());
  for (std::size_t i = 0; i < supports.size(); ++i) {
    best[i] = std::abs(residual[i]);
  }
  std::queue<std::size_t> queue;
  for (const auto index : seed_indices) {
    queue.push(index);
  }
  while (!queue.empty()) {
    const auto index = queue.front();
    queue.pop();
    for (const auto patch_id : members[index]) {
      if (activated[patch_id]) {
        continue;
      }
      if (++observed[patch_id] < shared_plane_support) {
        continue;
      }
      const auto & patch = patches[patch_id];
      std::vector<std::size_t> shared;
      for (const auto matched : patch.matched) {
        if (known[matched]) {
          shared.push_back(matched);
        }
      }
      if (shared.empty()) {
        continue;
      }
      Matrix shared_matrix(static_cast<Eigen::Index>(shared.size()), 3);
      for (Eigen::Index row = 0; row < shared_matrix.rows(); ++row) {
        const auto & point = supports[shared[static_cast<std::size_t>(row)]];
        shared_matrix(row, 0) = point.x;
        shared_matrix(row, 1) = point.y;
        shared_matrix(row, 2) = 1.0;
      }
      if (rank(shared_matrix) < 3) {
        continue;
      }
      bool slope_mismatch = false;
      for (const auto shared_index : shared) {
        const auto & old = coefficients[shared_index];
        if (std::hypot(old[0] - patch.plane[0], old[1] - patch.plane[1]) > max_slope) {
          slope_mismatch = true;
          break;
        }
      }
      if (slope_mismatch) {
        continue;
      }
      Plane prior{};
      if (!fitted(shared, supports, return_counts, obstacle_height, tolerance, prior,
        shared_plane_support)) {
        continue;
      }

      std::vector<std::size_t> matched;
      for (const auto candidate : patch.matched) {
        const auto & point = supports[candidate];
        const double predicted = point.x * prior[0] + point.y * prior[1] + prior[2];
        if (std::abs(point.z - predicted) < obstacle_height) {
          matched.push_back(candidate);
        }
      }
      activated[patch_id] = true;
      // The measured, connected patch also supplies ground height under
      // raised returns rejected by its plane fit. Keep this reference inside
      // the patch's measured 4 m support; do not extrapolate the near plane.
      for (const auto candidate : patch.support) {
        const auto & point = supports[candidate];
        const double distance = std::hypot(point.x-patch.x, point.y-patch.y);
        if (distance < reference_distance[candidate]) {
          reference_distance[candidate] = distance;
          references[candidate] = patch.plane;
        }
      }
      std::vector<std::size_t> newly_known;
      for (const auto candidate : matched) {
        const auto & point = supports[candidate];
        const double error = std::abs(point.z -
          (point.x * patch.plane[0] + point.y * patch.plane[1] + patch.plane[2]));
        if (error < best[candidate]) {
          coefficients[candidate] = patch.plane;
          best[candidate] = error;
        }
        if (!known[candidate]) {
          known[candidate] = true;
          newly_known.push_back(candidate);
        }
      }
      for (const auto candidate : newly_known) {
        queue.push(candidate);
      }
    }
  }

  for (const auto & point : eligible) {
    const CellKey key{static_cast<std::int64_t>(std::floor(point.x / .5)),
      static_cast<std::int64_t>(std::floor(point.y / .5))};
    const auto found = cell_indices.find(key);
    if (found == cell_indices.end()) {
      continue;
    }
    const auto index = found->second;
    const auto & local = coefficients[index];
    const double predicted = point.x * local[0] + point.y * local[1] + local[2];
    if (known[index]) {
      height[point.original] = predicted;
    } else if (std::isfinite(reference_distance[index])) {
      const auto & reference = references[index];
      height[point.original] = point.x*reference[0]+point.y*reference[1]+reference[2];
    }
    if (known[index] && std::abs(point.z - predicted) < tolerance) {
      output[point.original] = true;
    }
  }
  return py::make_tuple(result, heights);
}

PYBIND11_MODULE(_ground, module)
{
  module.def("ground_patch_model", &ground_patch_model,
    py::arg("points"), py::arg("plane"), py::arg("radius"),
    py::arg("obstacle_height"), py::arg("max_slope"));
  module.def("supported_ground_patches", [](
    py::array_t<double, py::array::c_style | py::array::forcecast> points,
    py::array_t<double, py::array::c_style | py::array::forcecast> plane,
    double radius, double obstacle_height, double max_slope) {
      auto model = ground_patch_model(points, plane, radius, obstacle_height, max_slope);
      return model[0].cast<py::array_t<bool>>();
    },
    py::arg("points"), py::arg("plane"), py::arg("radius"),
    py::arg("obstacle_height"), py::arg("max_slope"));
}
