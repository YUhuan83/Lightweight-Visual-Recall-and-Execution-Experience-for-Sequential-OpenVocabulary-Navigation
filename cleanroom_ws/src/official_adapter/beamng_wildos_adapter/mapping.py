"""Pure local occupancy construction for official graphnav_mapper input."""

from __future__ import annotations

import math
import numpy as np
from wildos_nav2_adapter.pointcloud import split_costmap_points
from wildos_nav2_adapter.vehicle_geometry import (
    SBR_FRONT_EXTENT,
    SBR_REAR_EXTENT,
    SBR_HALF_WIDTH,
)


def occupancy_from_points(
    points,
    resolution=0.5,
    width=80,
    height=80,
    ground_plane_radius=8.0,
    ground_clearance=0.25,
    sensor_origin_xy=(0.0, 0.0),
    clear_robot_footprint=False,
    carrier_model="ackermann",
    obstacle_min_height=0.10,
    ground_max_slope=0.35,
):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("points must be finite Nx3")
    carrier_model = str(carrier_model).strip().lower()
    if carrier_model not in ("ideal_point", "ackermann"):
        raise ValueError("carrier_model must be ideal_point or ackermann")
    if resolution <= 0 or width < 2 or height < 2:
        raise ValueError("invalid grid geometry")
    grid = np.full((height, width), -1, dtype=np.int8)
    free = np.zeros((height, width), dtype=bool)
    occupied = np.zeros((height, width), dtype=bool)
    sensor_origin_xy = np.asarray(sensor_origin_xy, dtype=float)
    if sensor_origin_xy.shape != (2,) or not np.isfinite(sensor_origin_xy).all():
        raise ValueError("sensor origin must be finite XY")
    center = (
        math.floor(sensor_origin_xy[0] / resolution + width / 2),
        math.floor(sensor_origin_xy[1] / resolution + height / 2),
    )
    if not (0 <= center[0] < width and 0 <= center[1] < height):
        raise ValueError("sensor origin must lie within the grid")
    has_vehicle_footprint = carrier_model == "ackermann"
    markings, clearing = split_costmap_points(
        points,
        min_height=0.1,
        max_height=3.0,
        filter_footprint=has_vehicle_footprint,
        front_extent=SBR_FRONT_EXTENT if has_vehicle_footprint else 0.0,
        rear_extent=SBR_REAR_EXTENT if has_vehicle_footprint else 0.0,
        half_width=SBR_HALF_WIDTH if has_vehicle_footprint else 0.0,
        voxel=0.2,
        max_points=8000,
        ground_plane_radius=ground_plane_radius,
        ground_clearance=ground_clearance,
        obstacle_min_height=obstacle_min_height,
        ground_max_slope=ground_max_slope,
    )

    def cell(x, y):
        return math.floor(x / resolution + width / 2), math.floor(
            y / resolution + height / 2
        )

    def ray(x1, y1):
        x0, y0 = center
        dx, sx = abs(x1 - x0), 1 if x0 < x1 else -1
        dy, sy = -abs(y1 - y0), 1 if y0 < y1 else -1
        error = dx + dy
        while True:
            if 0 <= x0 < width and 0 <= y0 < height:
                free[y0, x0] = True
            if x0 == x1 and y0 == y1:
                break
            twice = 2 * error
            if twice >= dy:
                error += dy
                x0 += sx
            if twice <= dx:
                error += dx
                y0 += sy

    for x, y, z in clearing:
        ix, iy = cell(x, y)
        if not (0 <= ix < width and 0 <= iy < height):
            continue
        ray(ix, iy)
    for x, y, z in markings:
        ix, iy = cell(x, y)
        if 0 <= ix < width and 0 <= iy < height:
            occupied[iy, ix] = True
    if clear_robot_footprint:
        if has_vehicle_footprint:
            # The occupied vehicle footprint is known locally even where its
            # body occludes LiDAR returns. Never expand this prior beyond the
            # measured body or override observed obstacles.
            rows, columns = np.indices(grid.shape)
            x = (columns + 0.5 - width / 2) * resolution
            y = (rows + 0.5 - height / 2) * resolution
            footprint = (
                (x >= -SBR_REAR_EXTENT)
                & (x <= SBR_FRONT_EXTENT)
                & (np.abs(y) <= SBR_HALF_WIDTH)
            )
            free[footprint] = True
        else:
            # A point carrier has no body area. Clear only the grid cell that
            # contains the reference point so its current graph node is usable.
            free[center[1], center[0]] = True
    grid[free] = 0
    # Obstacle endpoints always win over free-space rays from other returns.
    grid[occupied] = 100
    return grid


def safe_travel_distance(
    grid,
    resolution,
    origin_xy,
    target_local_xy,
    desired_travel,
    safety_margin=2.0,
    unknown_safety_margin=None,
):
    """Clamp travel before the first unsafe cell on the target ray.

    Occupied cells retain the full collision margin. For a confirmed visual
    target, callers may use a smaller buffer for unknown cells so the vehicle
    can advance in bounded steps and collect another local map before deciding
    whether the next segment is traversable.
    """
    grid = np.asarray(grid)
    tx, ty = map(float, target_local_xy)
    length = float(np.hypot(tx, ty))
    desired = min(max(float(desired_travel), 0.0), length)
    if length < 1e-6 or desired <= 0:
        return 0.0
    unknown_margin = (
        float(safety_margin)
        if unknown_safety_margin is None
        else float(unknown_safety_margin)
    )
    if (
        not math.isfinite(float(safety_margin))
        or safety_margin < 0.0
        or not math.isfinite(unknown_margin)
        or unknown_margin < 0.0
    ):
        raise ValueError("travel safety margins must be finite and non-negative")
    step = max(float(resolution) * 0.5, 0.05)
    distances = np.arange(step, desired + step, step)
    for distance in distances:
        x, y = tx * distance / length, ty * distance / length
        ix = math.floor((x - float(origin_xy[0])) / resolution)
        iy = math.floor((y - float(origin_xy[1])) / resolution)
        if not (0 <= ix < grid.shape[1] and 0 <= iy < grid.shape[0]):
            return max(0.0, distance - unknown_margin)
        cell = int(grid[iy, ix])
        if cell < 0:
            return max(0.0, distance - unknown_margin)
        if cell >= 50:
            return max(0.0, distance - safety_margin)
    return desired


def reachable_ring_candidate(
    grid,
    resolution,
    origin_xy,
    target_local_xy,
    radius,
    safety_margin=1.5,
    unknown_safety_margin=None,
    maintain_distance=False,
    accept=None,
):
    """Pick a collision-free point on a radius ring around a local target."""
    grid = np.asarray(grid)
    target = np.asarray(target_local_xy, dtype=float)
    distance = float(np.linalg.norm(target))
    if distance <= radius and not maintain_distance:
        candidate = np.zeros(2, dtype=float)
        return candidate if accept is None or accept(candidate) else None
    base = math.atan2(target[1], target[0])
    offsets = [
        0,
        15,
        -15,
        30,
        -30,
        45,
        -45,
        60,
        -60,
        75,
        -75,
        90,
        -90,
        105,
        -105,
        120,
        -120,
        135,
        -135,
        150,
        -150,
        165,
        -165,
        180,
    ]
    for degrees in offsets:
        angle = base + math.pi + math.radians(degrees)
        candidate = target + radius * np.array([math.cos(angle), math.sin(angle)])
        travel = float(np.linalg.norm(candidate))
        if travel < 0.1:
            if accept is None or accept(candidate):
                return candidate
            continue
        safe = safe_travel_distance(
            grid,
            resolution,
            origin_xy,
            candidate,
            travel,
            safety_margin,
            unknown_safety_margin,
        )
        if safe + 0.1 >= travel and (accept is None or accept(candidate)):
            return candidate
    return None
