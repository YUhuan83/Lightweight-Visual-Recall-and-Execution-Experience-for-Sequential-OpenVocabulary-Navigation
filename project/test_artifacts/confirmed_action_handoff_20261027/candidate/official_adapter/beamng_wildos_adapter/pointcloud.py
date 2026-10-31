"""Fast PointCloud2 x/y/z decoding and encoding."""
from __future__ import annotations
import numpy as np


def decode_xyz(msg):
    offsets = {f.name: int(f.offset) for f in msg.fields}
    if not {"x", "y", "z"}.issubset(offsets): raise ValueError("missing xyz")
    width, height = int(msg.width), int(msg.height)
    step, row_step = int(msg.point_step), int(msg.row_step)
    if step < max(offsets[name] for name in ("x", "y", "z")) + 4:
        raise ValueError("point_step too small")
    if width < 0 or height < 0 or row_step < width*step:
        raise ValueError("invalid row_step")
    raw = bytes(msg.data)
    if len(raw) < row_step*height: raise ValueError("truncated cloud")
    count = width*height
    if count == 0: return np.empty((0, 3), np.float32)
    endian = ">" if msg.is_bigendian else "<"
    out = np.empty((count, 3), np.float32)
    for i, name in enumerate(("x", "y", "z")):
        out[:, i] = np.ndarray((height, width), dtype=np.dtype(endian+"f4"),
                               buffer=raw, offset=offsets[name],
                               strides=(row_step, step)).reshape(-1)
    return out[np.isfinite(out).all(axis=1)]


def limit_points(points, max_points):
    """Retain samples across the whole sensor scan, not just its first sector."""
    points = np.asarray(points)
    if max_points < 1:
        raise ValueError("max_points must be positive")
    if len(points) <= max_points:
        return points
    indices = np.linspace(0, len(points)-1, max_points, dtype=np.intp)
    return points[indices]


def split_lidar_ranges(world_points, origin, body_rotation):
    """Independent budgets: 40 m navigation and 80 m object localization.

    Both use the acquisition pose and existing virtual LiDAR frame. Far
    returns must neither enter terrain nor displace local obstacle samples.
    """
    world_points = np.asarray(world_points)
    body_points = (world_points - origin) @ body_rotation
    radii = np.linalg.norm(body_points, axis=1)
    near = limit_points(np.flatnonzero(radii <= 40.), 30000)
    localization = limit_points(np.flatnonzero(radii <= 80.), 30000)
    return world_points[near], body_points[near], body_points[localization]
