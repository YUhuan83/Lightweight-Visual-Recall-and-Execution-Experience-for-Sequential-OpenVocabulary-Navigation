"""Nominal CameraInfo construction for the configured BeamNG camera."""
from __future__ import annotations

import math

import numpy as np


OPTICAL_TO_BODY = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])


def optical_rotation_from_body(body_rotation):
    """Map optical x-right/y-down/z-forward axes into the world frame."""
    rotation = np.asarray(body_rotation, dtype=float).reshape(3, 3)
    return rotation @ OPTICAL_TO_BODY


def nominal_camera_info(width: int, height: int, fov_y_deg: float, frame_id: str) -> dict:
    if width < 1 or height < 1 or not 0 < fov_y_deg < 180:
        raise ValueError("invalid camera geometry")
    fy = (height / 2.0) / math.tan(math.radians(fov_y_deg) / 2.0)
    fx = fy
    cx, cy = width / 2.0, height / 2.0
    return {
        "frame_id": frame_id,
        "width": int(width),
        "height": int(height),
        "distortion_model": "plumb_bob",
        "d": [0.0] * 5,
        "k": [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
        "r": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        "p": [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        "calibration_status": "NOMINAL_UNVERIFIED",
    }
