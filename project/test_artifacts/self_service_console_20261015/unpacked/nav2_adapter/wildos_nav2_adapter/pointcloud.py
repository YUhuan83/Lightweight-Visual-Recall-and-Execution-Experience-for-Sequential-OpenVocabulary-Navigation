"""Pure Python PointCloud2 decoding and bounded voxel downsampling."""
from __future__ import annotations

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


def local_ground_plane(points: np.ndarray, radius: float) -> np.ndarray | None:
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
    if len(unique) < 30:
        return None
    minima = np.full(len(unique), np.inf, dtype=float)
    np.minimum.at(minima, inverse, near[:, 2])
    coordinates = (unique.astype(float) + 0.5) * 0.5
    matrix = np.c_[coordinates, np.ones(len(coordinates))]
    keep = np.ones(len(coordinates), dtype=bool)
    for _ in range(4):
        if keep.sum() < 30:
            return None
        plane, _, rank, _ = np.linalg.lstsq(matrix[keep], minima[keep], rcond=None)
        if rank < 3:
            return None
        error = minima - matrix @ plane
        lower = np.quantile(error, 0.05) - 0.05
        upper = np.quantile(error, 0.65) + 0.05
        keep = (error >= lower) & (error <= upper)
    if (keep.sum() < 30 or np.hypot(plane[0], plane[1]) > 0.6 or
            np.percentile(np.abs(error[keep]), 90) > 0.15):
        return None
    return plane


def split_costmap_points(points: np.ndarray, *, min_height: float,
                         max_height: float, filter_footprint: bool,
                         front_extent: float, rear_extent: float,
                         half_width: float, voxel: float,
                         max_points: int, ground_plane_radius: float = 0.0,
                         ground_clearance: float = 0.25) -> tuple[np.ndarray, np.ndarray]:
    """Return obstacle markings and full-height first-hit clearing endpoints."""
    if filter_footprint:
        points = filter_robot_footprint(points, front_extent, rear_extent,
                                        half_width)
    # A high ray can pass above a low obstacle; do not let it erase the
    # obstacle's two-dimensional costmap cell.
    clearing = voxel_downsample(points[points[:, 2] <= max_height],
                                voxel, max_points)
    candidates = filter_height(points, min_height, max_height)
    if ground_plane_radius > 0:
        if ground_clearance <= 0:
            raise ValueError("ground_clearance must be positive")
        # Ground returns below min_height must participate in the fit. Fitting
        # only obstacle candidates can mistake a flat raised object for road.
        plane = local_ground_plane(points, ground_plane_radius)
        if plane is not None:
            nearby = np.hypot(candidates[:, 0], candidates[:, 1]) <= ground_plane_radius
            road_z = candidates[:, 0] * plane[0] + candidates[:, 1] * plane[1] + plane[2]
            candidates = candidates[~(nearby & (candidates[:, 2] - road_z < ground_clearance))]
    markings = voxel_downsample(candidates, voxel, max_points)
    return markings, clearing
