"""Coordinate helpers for BeamNG map/body/optical frames."""

from __future__ import annotations

import math
import numpy as np

OPTICAL_TO_BODY = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def xyz(value):
    return np.array([float(value.x), float(value.y), float(value.z)])


def body_rotation(forward, up):
    f = xyz(forward)
    f /= max(float(np.linalg.norm(f)), 1e-9)
    u = xyz(up)
    u /= max(float(np.linalg.norm(u)), 1e-9)
    right = np.cross(f, u)
    right /= max(float(np.linalg.norm(right)), 1e-9)
    left = -right
    u = np.cross(f, left)
    u /= max(float(np.linalg.norm(u)), 1e-9)
    return np.column_stack((f, left, u))


def quaternion_matrix(q):
    x, y, z, w = (float(v) for v in q)
    n = x * x + y * y + z * z + w * w
    if n <= 1e-12:
        raise ValueError("zero quaternion")
    s = 2.0 / n
    return np.array(
        [
            [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
            [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
            [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
        ]
    )


def matrix_quaternion(m):
    m = np.asarray(m, dtype=float)
    trace = float(np.trace(m))
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        return (
            (m[2, 1] - m[1, 2]) / s,
            (m[0, 2] - m[2, 0]) / s,
            (m[1, 0] - m[0, 1]) / s,
            0.25 * s,
        )
    idx = int(np.argmax(np.diag(m)))
    if idx == 0:
        s = math.sqrt(1 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        return (
            0.25 * s,
            (m[0, 1] + m[1, 0]) / s,
            (m[0, 2] + m[2, 0]) / s,
            (m[2, 1] - m[1, 2]) / s,
        )
    if idx == 1:
        s = math.sqrt(1 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        return (
            (m[0, 1] + m[1, 0]) / s,
            0.25 * s,
            (m[1, 2] + m[2, 1]) / s,
            (m[0, 2] - m[2, 0]) / s,
        )
    s = math.sqrt(1 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
    return (
        (m[0, 2] + m[2, 0]) / s,
        (m[1, 2] + m[2, 1]) / s,
        0.25 * s,
        (m[1, 0] - m[0, 1]) / s,
    )


def optical_rotation(body):
    return np.asarray(body) @ OPTICAL_TO_BODY


def shifted_body_position(position, rotation, offset):
    """World position of a fixed body point; offset is in body coordinates."""
    offset = np.asarray(offset, dtype=float)
    if offset.shape != (3,) or not np.isfinite(offset).all():
        raise ValueError("base_offset must contain three finite numbers")
    return np.asarray(position, dtype=float) + np.asarray(rotation) @ offset


def rotation_velocity(previous, current, dt):
    """World angular velocity from two measured rotations, shortest SO(3) arc."""
    if previous is None or not 0 < dt <= 0.5:
        return np.zeros(3)
    q = np.asarray(matrix_quaternion(np.asarray(current) @ np.asarray(previous).T))
    q /= np.linalg.norm(q)
    if q[3] < 0:
        q = -q
    sine = np.linalg.norm(q[:3])
    if sine < 1e-9:
        return 2.0 * q[:3] / dt
    return q[:3] * (2.0 * math.atan2(sine, q[3]) / (sine * dt))


def shifted_body_velocity(velocity, rotation, offset, angular_world):
    """Twist at the shifted point, including omega cross r, in body axes."""
    rotation = np.asarray(rotation)
    world = np.asarray(velocity) + np.cross(angular_world, rotation @ offset)
    return world @ rotation


def forward_xy(q):
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    return (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y + z * w))


def standoff_xy(robot_xy, target_xy, distance, maintain_distance=False):
    dx = float(target_xy[0]) - float(robot_xy[0])
    dy = float(target_xy[1]) - float(robot_xy[1])
    length = math.hypot(dx, dy)
    if length <= distance and not maintain_distance:
        return float(robot_xy[0]), float(robot_xy[1])
    if length < 1e-6:
        return float(target_xy[0]) - distance, float(target_xy[1])
    scale = (length - distance) / length
    return float(robot_xy[0]) + dx * scale, float(robot_xy[1]) + dy * scale
