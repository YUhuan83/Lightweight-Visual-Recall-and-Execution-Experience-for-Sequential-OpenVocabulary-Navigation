"""Fixture-derived geometry and capture readers shared by the integration driver.

All numbers come from sensor_calibration/fixture.py geometry() and fit_intrinsics()
defaults; nothing here acquires from a simulator or invents acquisition timestamps.
"""
import hashlib
import json
import math
from pathlib import Path

import numpy as np

WORLD_FRAME = "smallgrid_world"
CAMERA_FRAME = "static_optical"
LOCAL_CLOUD_FRAME = "ego_lidar_raw"
TIME_SOURCE = "beamng_host_monotonic"

WIDTH, HEIGHT = 960, 640
FOV_Y_DEG = 60.0
STATIC_CAMERA_POS = (0.0, 0.0, 2.35)
STATIC_CAMERA_DIR = (0.0, -1.0, 0.0)
STATIC_CAMERA_UP = (0.0, 0.0, 1.0)

# Fixture object centres in world metres, from fixture.geometry().
FIXTURE_BOXES = {
    "red": [-3.0, -6.0, 0.85],
    "blue": [3.0, -8.0, 1.1],
    "green": [-3.0, -11.0, 0.6],
}


def nominal_k(width=WIDTH, height=HEIGHT, fov_y=FOV_Y_DEG):
    focal = height / (2.0 * math.tan(math.radians(fov_y) / 2.0))
    return [[focal, 0.0, width / 2.0],
            [0.0, focal, height / 2.0],
            [0.0, 0.0, 1.0]]


def static_world_to_camera(position=STATIC_CAMERA_POS):
    """Rows are camera axes (right, down, forward) expressed in world coordinates.

    Matches fixture.coordinates(): forward = dir, right = dir x up, down = forward x right.
    """
    forward = np.asarray(STATIC_CAMERA_DIR, float)
    forward /= np.linalg.norm(forward)
    up = np.asarray(STATIC_CAMERA_UP, float)
    right = np.cross(forward, up)
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    rotation = np.stack((right, down, forward))
    translation = -rotation @ np.asarray(position, float)
    return rotation, translation


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def capture_paths(capture):
    capture = Path(capture)
    return {"dir": capture, "static": capture / "static.json",
            "mounted": capture / "mounted.json", "lidar": capture / "lidar.json",
            "static_png": capture / "static.png", "mounted_png": capture / "mounted.png",
            "lidar_npy": capture / "lidar.npy"}


def acquisition_stamp_ns(manifest):
    """Host monotonic seconds -> integer nanoseconds. Never called 'simulation time'."""
    return int(round(float(manifest["metadata"]["host_monotonic_start"]) * 1e9))


def load_capture(capture):
    paths = capture_paths(capture)
    for key in ("static", "mounted", "lidar"):
        if not paths[key].is_file():
            raise FileNotFoundError(paths[key])
    return {"paths": paths,
            "static": read_json(paths["static"]),
            "mounted": read_json(paths["mounted"]),
            "lidar": read_json(paths["lidar"])}


def verify_se3(rotation, translation):
    rotation = np.asarray(rotation, float)
    translation = np.asarray(translation, float)
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-9, rtol=0):
        raise ValueError("rotation is not orthonormal")
    if abs(np.linalg.det(rotation) - 1.0) > 1e-9:
        raise ValueError("rotation determinant is not +1")
    if translation.shape != (3,) or not np.isfinite(translation).all():
        raise ValueError("translation must be a finite 3-vector")
    return rotation, translation
