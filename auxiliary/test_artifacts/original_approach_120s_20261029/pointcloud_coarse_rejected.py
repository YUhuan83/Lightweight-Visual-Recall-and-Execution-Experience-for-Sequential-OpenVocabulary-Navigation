"""Pure Python PointCloud2 decoding and bounded voxel downsampling."""
from __future__ import annotations

from collections import deque
from typing import Iterable

import numpy as np


def field_offsets(fields: Iterable[object]) -> dict[str, int]:
    return {str(field.name): int(field.offset) for field in fields}


def decode_xyz(message) -> np.ndarray:
    """Decode x/y/z FLOAT32 fields from a sensor_msgs PointCloud2 message."""
    offsets = field_offsets(message.fields)
    required = {"x", "y", "z"}
    if not required.issubset(offsets):
        raise ValueError("PointCloud2 is missing x/y/z fields")
    if int(message.point_step) < max(offsets[n] for n in required) + 4:
        raise ValueError("PointCloud2 point_step is too small")
    width, height = int(message.width), int(message.height)
    step, row_step = int(message.point_step), int(message.row_step)
    if width < 0 or height < 0 or row_step < width * step:
        raise ValueError("PointCloud2 row_step is invalid")
    raw = bytes(message.data)
    expected = row_step * height
    if len(raw) < expected:
        raise ValueError("PointCloud2 data is truncated")
    count = width * height
    if count == 0:
        return np.empty((0, 3), dtype=np.float32)
    endian = ">" if bool(message.is_bigendian) else "<"
    points = np.empty((count, 3), dtype=np.float32)
    for index, name in enumerate(("x", "y", "z")):
        offset = offsets[name]
        points[:, index] = np.ndarray(
            shape=(height, width), dtype=np.dtype(endian + "f4"), buffer=raw,
            offset=offset, strides=(row_step, step)).reshape(-1)
    return points[np.isfinite(points).all(axis=1)]


def filter_height(points: np.ndarray, min_height: float,
                  max_height: float) -> np.ndarray:
    """Filter by z relative to the cloud's robot-base frame."""
    points = np.asarray(points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must be Nx3")
    if not np.isfinite(points).all():
        raise ValueError("points must be finite")
    if not np.isfinite((min_height, max_height)).all() or min_height > max_height:
        raise ValueError("height bounds must be finite and ordered")
    return points[(points[:, 2] >= min_height) & (points[:, 2] <= max_height)]


def filter_robot_footprint(points: np.ndarray, front_extent: float,
                           rear_extent: float,
                           half_width: float) -> np.ndarray:
    """Remove returns inside the robot body footprint in base-frame XY."""
    points = np.asarray(points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must be Nx3")
    if not np.isfinite(points).all():
        raise ValueError("points must be finite")
    if (not np.isfinite((front_extent, rear_extent, half_width)).all()
            or front_extent < 0 or rear_extent < 0 or half_width < 0):
        raise ValueError("footprint extents must be finite and non-negative")
    inside = (
        (points[:, 0] >= -rear_extent)
        & (points[:, 0] <= front_extent)
        & (np.abs(points[:, 1]) <= half_width)
    )
    return points[~inside]


def voxel_downsample(points: np.ndarray, voxel: float, max_points: int) -> np.ndarray:
    """Keep one point per voxel, bounded by max_points, deterministically."""
    points = np.asarray(points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must be Nx3")
    if not np.isfinite(points).all():
        raise ValueError("points must be finite")
    if voxel <= 0 or max_points < 1:
        raise ValueError("voxel must be positive and max_points must be positive")
    keys = np.floor(points / float(voxel)).astype(np.int64)
    _, first = np.unique(keys, axis=0, return_index=True)
    selected = points[np.sort(first)]
    if len(selected) > max_points:
        indices = np.linspace(0, len(selected) - 1, max_points, dtype=np.intp)
        selected = selected[indices]
    return selected


def local_ground_plane(points: np.ndarray, radius: float,
                       minimum_cells: int = 30) -> np.ndarray | None:
    """Fit a nearby road plane to lower returns in 0.5 m XY cells.

    Cell minima keep sparse vertical objects from dominating the fit. The fit
    rejects high residuals iteratively; a poor or undersampled fit is ignored.
    """
    if radius <= 0 or len(points) < 100:
        return None
    near = points[(np.hypot(points[:, 0], points[:, 1]) <= radius)
                  & (points[:, 2] >= -2.0) & (points[:, 2] <= 1.2)]
    if len(near) < 100:
        return None
    cells = np.floor(near[:, :2] / 0.5).astype(np.int32)
    unique, inverse = np.unique(cells, axis=0, return_inverse=True)
    if len(unique) < minimum_cells:
        return None
    minima = np.full(len(unique), np.inf, dtype=float)
    np.minimum.at(minima, inverse, near[:, 2])
    # A cell minimum belongs to a measured return, not the cell center. Moving
    # only XY biases the road height on slopes and changes obstacle residuals.
    is_minimum = near[:, 2] == minima[inverse]
    minimum_indices = np.full(len(unique), len(near), dtype=np.int64)
    np.minimum.at(minimum_indices, inverse[is_minimum], np.flatnonzero(is_minimum))
    coordinates = near[minimum_indices, :2]
    matrix = np.c_[coordinates, np.ones(len(coordinates))]
    keep = np.ones(len(coordinates), dtype=bool)
    for _ in range(4):
        if keep.sum() < minimum_cells:
            return None
        plane, _, rank, _ = np.linalg.lstsq(matrix[keep], minima[keep], rcond=None)
        if rank < 3:
            return None
        error = minima - matrix @ plane
        lower = np.quantile(error, 0.05) - 0.05
        upper = np.quantile(error, 0.65) + 0.05
        keep = (error >= lower) & (error <= upper)
    if (keep.sum() < minimum_cells or np.hypot(plane[0], plane[1]) > 0.6 or
            np.percentile(np.abs(error[keep]), 90) > 0.15):
        return None
    return plane


def connected_ground_returns(points: np.ndarray, plane: np.ndarray,
                             radius: float, obstacle_min_height: float,
                             max_slope: float = .35) -> np.ndarray:
    """Confirm a locally continuous lower surface, without bridging missing data.

    The large plane supplies initial nearby support only. It cannot describe
    a curved road. Grow through adjacent measured map-resolution cells using actual
    minimum-return locations and a bounded rise/run; require support in both
    axes before confirming a surface. A raised top separated by a step or an
    unmeasured gap cannot be normalised away as a new ground plane.

    This removes ground from obstacle observations; it does not certify body
    clearance or mark unknown space traversable. Terrain/footprint checks are
    still required downstream. Unconfirmed returns keep the original gate.
    """
    if not np.isfinite(max_slope) or max_slope < 0:
        raise ValueError('ground max_slope must be finite and non-negative')
    confirmed = np.zeros(len(points), dtype=bool)
    near = np.flatnonzero((np.linalg.norm(points[:, :2], axis=1) <= radius)
                         & (points[:, 2] >= -2.) & (points[:, 2] <= 1.2))
    if not len(near):
        return confirmed
    p = points[near]
    # Canonical extrema preserve permutation invariance and measured XY/Z.
    order = np.lexsort((p[:, 2], p[:, 1], p[:, 0]))
    p, near = p[order], near[order]
    scaled = p[:, :2] / .5
    nearest = np.rint(scaled)
    tolerance = 4. * np.finfo(float).eps * np.maximum(1., np.abs(scaled))
    scaled = np.where(np.abs(scaled - nearest) <= tolerance, nearest, scaled)
    cells, inverse = np.unique(np.floor(scaled).astype(np.int64), axis=0,
                               return_inverse=True)
    lows = np.full(len(cells), np.inf)
    np.minimum.at(lows, inverse, p[:, 2])
    selected = np.full(len(cells), len(p), dtype=np.int64)
    mask = p[:, 2] == lows[inverse]
    np.minimum.at(selected, inverse[mask], np.flatnonzero(mask))
    support = p[selected]
    residual = support[:, 2] - (support[:, :2] @ plane[:2] + plane[2])
    ground = ((np.abs(residual) <= min(.05, obstacle_min_height / 2.))
              & (np.linalg.norm(support[:, :2], axis=1) <= min(4., radius)))
    if not np.any(ground):
        return confirmed
    offset = cells.min(axis=0)
    stride = int(cells[:, 1].max() - offset[1] + 3)
    packed = (cells[:, 0] - offset[0]) * stride + cells[:, 1] - offset[1]
    # len(cells) is a non-ground sentinel, avoiding negative indexing.
    adjacency = np.full((len(cells), 4), len(cells), dtype=np.int64)
    for direction, (dx, dy) in enumerate(((1, 0), (-1, 0), (0, 1), (0, -1))):
        target = cells + (dx, dy)
        keys = (target[:, 0] - offset[0]) * stride + target[:, 1] - offset[1]
        positions = np.searchsorted(packed, keys)
        left = np.flatnonzero(positions < len(packed))
        right = positions[left]
        exact = packed[right] == keys[left]
        left, right = left[exact], right[exact]
        distance = np.linalg.norm(support[left, :2] - support[right, :2], axis=1)
        rise = lows[left] - lows[right]
        plane_rise = (support[left, :2] - support[right, :2]) @ plane[:2]
        continuous = ((np.abs(rise) <= max_slope * distance + 1.e-6)
                      & (np.abs(rise - plane_rise) < obstacle_min_height - 1.e-6))
        adjacency[left[continuous], direction] = right[continuous]
    reached = np.r_[ground, False]
    queue = deque(np.flatnonzero(ground))
    while queue:
        for neighbour in adjacency[queue.popleft()]:
            if neighbour < len(cells) and not reached[neighbour]:
                reached[neighbour] = True
                queue.append(neighbour)
    neighbours = reached[adjacency]
    surface = reached[:-1] & neighbours[:, :2].any(axis=1) & neighbours[:, 2:].any(axis=1)
    # The minimum retains its measured XY. Compensate the road tilt within
    # each 0.5 m map cell before testing an individual return's height.
    reference = lows[inverse] + (p[:, :2] - support[inverse, :2]) @ plane[:2]
    confirmed[near] = surface[inverse] & (p[:, 2] - reference < obstacle_min_height)
    return confirmed


def supported_far_ground_returns(points: np.ndarray, plane: np.ndarray,
                                 candidates: np.ndarray, radius: float,
                                 obstacle_height: float,
                                 max_slope: float) -> np.ndarray:
    """Remove far markings only with measured local surface corroboration.

    The nearby plane supplies a height consistency check, not a far ground
    certificate. Fit the acquired points around each far patch independently;
    missing support, raised tops and inconsistent surfaces retain their marks.
    No new clearing endpoints or free-space observations are created.
    """
    confirmed = np.zeros(len(points), dtype=bool)
    residual = points[:, 2] - (points[:, :2] @ plane[:2] + plane[2])
    indices = np.flatnonzero(candidates & (np.abs(residual) < obstacle_height))
    if not len(indices):
        return confirmed
    # One fit per nearby group bounds work without extending a fitted patch.
    _, groups = np.unique(np.floor(points[indices, :2] / radius).astype(np.int64),
                          axis=0, return_inverse=True)
    patch_radius = min(4., radius)
    for group in range(int(groups.max()) + 1):
        selected = indices[groups == group]
        center = points[selected, :2].mean(axis=0)
        shifted = points - np.r_[center, 0.]
        # Four independent cells per plane coefficient, plus the existing
        # >=100 returns, full-rank and robust residual checks. The nearby fit
        # keeps its original 30-cell requirement.
        local = local_ground_plane(shifted, patch_radius, minimum_cells=12)
        if local is None or np.linalg.norm(local[:2] - plane[:2]) > max_slope:
            continue
        reference = float(center @ plane[:2] + plane[2])
        if abs(local[2] - reference) >= obstacle_height:
            continue
        delta = shifted[selected, :2]
        height = points[selected, 2] - (delta @ local[:2] + local[2])
        ground = ((np.linalg.norm(delta, axis=1) <= patch_radius)
                  & (np.abs(height) < min(.05, obstacle_height / 2.)))
        confirmed[selected[ground]] = True
    return confirmed


def split_costmap_points(points: np.ndarray, *, min_height: float,
                         max_height: float, filter_footprint: bool,
                         front_extent: float, rear_extent: float,
                         half_width: float, voxel: float,
                         max_points: int, ground_plane_radius: float = 0.0,
                         ground_clearance: float = 0.25,
                         obstacle_min_height: float | None = None,
                         ground_max_slope: float = .35
                         ) -> tuple[np.ndarray, np.ndarray]:
    """Return obstacle markings and full-height first-hit clearing endpoints.

    When ``obstacle_min_height`` is provided, points inside the ground-fit
    radius are classified by their height above the local road plane instead
    of their absolute base-frame Z.  This keeps a low kerb/rock that the
    vehicle can hit even when its top is below the legacy ``min_height`` gate.
    ``obstacle_min_height=None`` preserves the previous body-frame behaviour.
    """
    if filter_footprint:
        points = filter_robot_footprint(points, front_extent, rear_extent,
                                        half_width)
    # A high ray can pass above a low obstacle; do not let it erase the
    # obstacle's two-dimensional costmap cell.
    clearing = voxel_downsample(points[points[:, 2] <= max_height],
                                voxel, max_points)
    if obstacle_min_height is None:
        candidates = filter_height(points, min_height, max_height)
        if ground_plane_radius > 0:
            if ground_clearance <= 0:
                raise ValueError("ground_clearance must be positive")
            # Ground returns below min_height must participate in the fit.
            # Fitting only obstacle candidates can mistake a flat raised
            # object for road.
            plane = local_ground_plane(points, ground_plane_radius)
            if plane is not None:
                nearby = np.hypot(candidates[:, 0], candidates[:, 1]) <= ground_plane_radius
                road_z = (candidates[:, 0] * plane[0]
                          + candidates[:, 1] * plane[1] + plane[2])
                candidates = candidates[
                    ~(nearby & (candidates[:, 2] - road_z < ground_clearance))]
    else:
        if not np.isfinite(obstacle_min_height) or obstacle_min_height <= 0:
            raise ValueError("obstacle_min_height must be positive and finite")
        if not np.isfinite(ground_max_slope) or ground_max_slope < 0:
            raise ValueError("ground_max_slope must be finite and non-negative")
        # Start far returns with the legacy body-frame gate; remove a marking
        # only after independent measured local support corroborates ground.
        # The relative gate is deliberately not
        # derived from min_height: a vehicle can hit a low object whose top
        # never reaches the base-frame threshold.
        plane = (local_ground_plane(points, ground_plane_radius)
                 if ground_plane_radius > 0 else None)
        if plane is None:
            candidates = filter_height(points, min_height, max_height)
        else:
            residual = points[:, 2] - (points[:, 0] * plane[0]
                                       + points[:, 1] * plane[1]
                                       + plane[2])
            nearby = (np.hypot(points[:, 0], points[:, 1])
                      <= ground_plane_radius)
            near_obstacle = (nearby & np.isfinite(residual)
                             & (residual >= obstacle_min_height)
                             & (points[:, 2] <= max_height))
            far_body = ((~nearby) & (points[:, 2] >= min_height)
                        & (points[:, 2] <= max_height))
            if np.any(near_obstacle | far_body):
                # The nearby plane seeds ground. Actual measured continuity
                # can support farther returns without extrapolating that plane.
                radius = max(ground_plane_radius, float(np.hypot(points[:, 0], points[:, 1]).max()))
                ground = connected_ground_returns(points, plane, radius,
                                                   obstacle_min_height, ground_max_slope)
                near_obstacle &= ~ground
                far_body &= ~ground
            if np.any(far_body):
                far_body &= ~supported_far_ground_returns(
                    points, plane, far_body, ground_plane_radius,
                    obstacle_min_height, ground_max_slope)
            candidates = points[near_obstacle | far_body]
    markings = voxel_downsample(candidates, voxel, max_points)
    return markings, clearing
