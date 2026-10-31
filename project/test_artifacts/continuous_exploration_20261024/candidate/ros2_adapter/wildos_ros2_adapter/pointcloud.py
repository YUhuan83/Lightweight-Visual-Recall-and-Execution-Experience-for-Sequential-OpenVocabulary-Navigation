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
    count = int(message.width) * int(message.height)
    if count < 0 or int(message.row_step) < count * int(message.point_step):
        raise ValueError("PointCloud2 row_step is invalid")
    raw = bytes(message.data)
    expected = count * int(message.point_step)
    if len(raw) < expected:
        raise ValueError("PointCloud2 data is truncated")
    endian = ">" if bool(message.is_bigendian) else "<"
    step = int(message.point_step)
    points = np.empty((count, 3), dtype=np.float32)
    for index, name in enumerate(("x", "y", "z")):
        offset = offsets[name]
        points[:, index] = np.ndarray(
            shape=(count,), dtype=np.dtype(endian + "f4"), buffer=raw,
            offset=offset, strides=(step,))
    return points[np.isfinite(points).all(axis=1)]


def voxel_downsample(points: np.ndarray, voxel: float, max_points: int) -> np.ndarray:
    """Keep one point per voxel, bounded by max_points, deterministically."""
    points = np.asarray(points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must be Nx3")
    if not np.isfinite(points).all():
        raise ValueError("points must be finite")
    if voxel <= 0 or max_points < 1:
        raise ValueError("voxel must be positive and max_points must be positive")
    if len(points) <= max_points:
        return points.copy()
    keys = np.floor(points / float(voxel)).astype(np.int64)
    _, first = np.unique(keys, axis=0, return_index=True)
    selected = points[np.sort(first)]
    if len(selected) > max_points:
        selected = selected[:max_points]
    return selected
