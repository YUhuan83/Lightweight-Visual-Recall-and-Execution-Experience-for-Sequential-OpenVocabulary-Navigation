"""Standalone, ROS-free port of WildOS coarse goal-query localization.

Faithful to wildos_port workspace sources:
  triangulation3d/bbox_generator.py      (ray through bounding-box centre)
  triangulation3d/particle_generator.py  (pixels-in-mask + uniform or hypothesis-depth particles)
  triangulation3d/triangulator.py        (inverse-distance ray-weighted / minimum triangulation)
  visual_navigation/.../obj_mask_triangulation.py (LiDAR-in-mask median-depth lock)

Conventions: K is a standard pinhole (x right, y down, z forward); (R_wc, t_wc)
maps camera coordinates to world, p_world = R_wc @ p_cam + t_wc.
"""
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

import numpy as np

DEPTH_MIN, DEPTH_MAX = 1.0, 100.0
HYPOTHESIS_STD_M = 5.0


@dataclass
class Ray:
    origin: np.ndarray
    direction: np.ndarray

    def __post_init__(self):
        self.origin = np.asarray(self.origin, float).reshape(3)
        self.direction = np.asarray(self.direction, float).reshape(3)
        norm = np.linalg.norm(self.direction)
        if not np.isfinite(norm) or norm == 0:
            raise ValueError("Ray direction must be a finite nonzero vector")
        self.direction = self.direction / norm

    def distance_to_points(self, points):
        points = np.asarray(points, float)
        vec = points - self.origin.reshape(1, 3)
        along = vec @ self.direction
        projection = along[:, None] * self.direction.reshape(1, 3) + self.origin.reshape(1, 3)
        distances = np.linalg.norm(points - projection, axis=1)
        distances[along <= 0] = np.inf  # points behind the ray origin
        return distances


def ray_from_bbox(bbox, K, R_wc, t_wc) -> Ray:
    """Ray from the camera centre through the bounding-box centre (image pixels)."""
    x_min, y_min, x_max, y_max = (float(v) for v in bbox)
    centre = np.array([(x_min + x_max) / 2.0, (y_min + y_max) / 2.0, 1.0])
    direction_cam = np.linalg.inv(K) @ centre
    direction_world = np.asarray(R_wc, float) @ direction_cam
    return Ray(origin=np.asarray(t_wc, float).reshape(3), direction=direction_world)


def bbox_from_mask(mask) -> np.ndarray:
    ys, xs = np.nonzero(mask)
    if len(xs) == 0:
        raise ValueError("Empty mask has no bounding box")
    return np.array([xs.min(), ys.min(), xs.max() + 1, ys.max() + 1], dtype=float)


def rotation_from_angles(angle_x, angle_z):
    """Small-angle rotation about world X then Z (Rodrigues), matching euler 'xyz' intent."""
    def axis_rotation(axis, angle):
        axis = np.asarray(axis, float)
        axis = axis / np.linalg.norm(axis)
        c, s = math.cos(angle), math.sin(angle)
        cross = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        return c * np.eye(3) + s * cross + (1 - c) * np.outer(axis, axis)
    return axis_rotation([0, 0, 1], angle_z) @ axis_rotation([1, 0, 0], angle_x)


def add_odom_drift(R_wc, t_wc, rng, t_drift_std=1.0, r_drift_std_deg=2.0):
    """Add translation and X/Z rotation noise to the camera pose (official default off)."""
    t_noise = rng.normal(0.0, t_drift_std, size=3)
    angles = rng.normal(0.0, math.radians(r_drift_std_deg), size=2)
    delta = rotation_from_angles(angles[0], angles[1])
    return delta @ np.asarray(R_wc, float), np.asarray(t_wc, float) + t_noise


def sample_depths(ray, R_wc, t_wc, depth_range, num, rng,
                  depth_hypothesis: Optional[Dict] = None,
                  distribution: str = "uniform", hypothesis_std=HYPOTHESIS_STD_M):
    """Uniform depth, or depths around a 2D/3D hypothesis (official get_gaussian_samples)."""
    if depth_hypothesis is None or distribution == "uniform":
        return rng.uniform(depth_range[0], depth_range[1], size=num)
    location = np.asarray(depth_hypothesis["location"], float)
    t_wc = np.asarray(t_wc, float).reshape(3)
    if location.shape == (3,):
        distance_3d = float(np.linalg.norm(location - t_wc))
    elif location.shape == (2,):
        distance_2d = float(np.linalg.norm(location - t_wc[:2]))
        cos_xy = math.sqrt(max(0.0, 1.0 - float(ray.direction[2]) ** 2))
        if cos_xy <= 1e-9:
            raise ValueError("Vertical ray makes a 2D hypothesis ambiguous")
        distance_3d = distance_2d / cos_xy
    else:
        raise ValueError("depth_hypothesis['location'] must be a 2D or 3D point")
    forward_world = np.asarray(R_wc, float) @ np.array([0.0, 0.0, 1.0])
    forward_world = forward_world / np.linalg.norm(forward_world)
    depth_3d = distance_3d * float(np.dot(ray.direction, forward_world))
    if distribution == "gaussian_hyp":
        samples = rng.normal(depth_3d, hypothesis_std, size=num)
    elif distribution == "uniform_hyp":
        samples = rng.uniform(max(depth_3d - hypothesis_std, depth_range[0]),
                              min(depth_3d + hypothesis_std, depth_range[1]), size=num)
    else:
        raise ValueError("distribution must be uniform, gaussian_hyp or uniform_hyp")
    return np.clip(samples, depth_range[0], depth_range[1])


def generate_particles(mask, K, R_wc, t_wc, num_particles, depth_range, rng,
                       ray: Optional[Ray] = None, depth_hypothesis: Optional[Dict] = None,
                       distribution: str = "uniform", hypothesis_std=HYPOTHESIS_STD_M):
    """Sample pixels inside the mask and depths, back-project to world."""
    ys, xs = np.nonzero(np.asarray(mask))
    if len(xs) == 0:
        raise ValueError("Empty mask cannot generate particles")
    pick = rng.integers(0, len(xs), size=num_particles)
    u = xs[pick].astype(float)
    v = ys[pick].astype(float)
    if ray is None:
        ray = ray_from_bbox(bbox_from_mask(mask), K, R_wc, t_wc)
    depth = sample_depths(ray, R_wc, t_wc, depth_range, num_particles, rng,
                          depth_hypothesis=depth_hypothesis, distribution=distribution,
                          hypothesis_std=hypothesis_std)
    pixels = np.vstack((u, v, np.ones(num_particles)))
    points_cam = (np.linalg.inv(np.asarray(K, float)) @ pixels) * depth.reshape(1, -1)
    points_world = (np.asarray(R_wc, float) @ points_cam + np.asarray(t_wc, float).reshape(3, 1)).T
    return points_world


def triangulate(points_world, rays: Sequence[Ray], method="weighted") -> np.ndarray:
    """Inverse-distance ray weighting (official WEIGHTED) or nearest-to-all-rays point (MINIMUM)."""
    points = np.asarray(points_world, float)
    if points.ndim != 2 or points.shape[1] != 3 or not len(points):
        raise ValueError("Particles must be a nonempty Nx3 array")
    weights = np.zeros(len(points))
    total_distance = np.zeros(len(points))
    for ray in rays:
        distances = ray.distance_to_points(points)
        total_distance = total_distance + np.where(np.isfinite(distances), distances, 0.0)
        with np.errstate(divide="ignore"):
            contribution = 1.0 / (distances + 1e-3)
        contribution[~np.isfinite(contribution)] = 0.0
        weights += contribution
    if method == "minimum":
        return points[int(np.argmin(total_distance))]
    if method != "weighted":
        raise ValueError("method must be weighted or minimum")
    total = weights.sum()
    if total <= 0:
        raise ValueError("All particles are behind every ray; cannot triangulate")
    return (points * (weights / total)[:, None]).sum(axis=0)


def localize_in_range(cloud_world, mask, K, R_wc, t_wc, min_points=150):
    """LiDAR-in-mask median-depth lock (the 'trivial' in-range case). Returns world XYZ or None."""
    cloud_world = np.asarray(cloud_world, float)
    if cloud_world.ndim != 2 or cloud_world.shape[1] != 3:
        raise ValueError("cloud_world must be Nx3")
    R_cw = np.asarray(R_wc, float).T
    t_cw = -R_cw @ np.asarray(t_wc, float).reshape(3)
    points_cam = cloud_world @ R_cw.T + t_cw
    front = points_cam[:, 2] > 0.1
    points_cam = points_cam[front]
    if len(points_cam) == 0:
        return None
    pixels = (np.asarray(K, float) @ points_cam.T).T
    uv = pixels[:, :2] / pixels[:, 2:3]
    h, w = np.asarray(mask).shape
    ui = uv[:, 0].astype(np.int64)
    vi = uv[:, 1].astype(np.int64)
    inside = (ui >= 0) & (ui < w) & (vi >= 0) & (vi < h)
    points_cam, ui, vi = points_cam[inside], ui[inside], vi[inside]
    if len(points_cam) == 0:
        return None
    selected = np.asarray(mask)[vi, ui].astype(bool)
    points_cam = points_cam[selected]
    if len(points_cam) < max(1, int(min_points)):
        return None
    distances = np.linalg.norm(points_cam, axis=1)
    median_point_cam = points_cam[np.argsort(distances)[len(distances) // 2]]
    return np.asarray(R_wc, float) @ median_point_cam + np.asarray(t_wc, float).reshape(3)


@dataclass
class View:
    mask: np.ndarray
    K: np.ndarray
    R_wc: np.ndarray
    t_wc: np.ndarray


def localize(views: List[View], cloud_world=None, num_particles=1000,
             depth_range=(DEPTH_MIN, DEPTH_MAX), min_lidar_points=150, seed=42,
             depth_hypothesis: Optional[Dict] = None, distribution="uniform",
             hypothesis_std=HYPOTHESIS_STD_M, method="weighted"):
    """Return (position_world, method). Prefers the LiDAR lock, else particle triangulation."""
    if cloud_world is not None:
        for view in views:
            position = localize_in_range(cloud_world, view.mask, view.K, view.R_wc, view.t_wc,
                                         min_points=min_lidar_points)
            if position is not None:
                return position, "lidar_median_depth"
    rng = np.random.default_rng(seed)
    particles, rays = [], []
    for view in views:
        ray = ray_from_bbox(bbox_from_mask(view.mask), view.K, view.R_wc, view.t_wc)
        rays.append(ray)
        particles.append(generate_particles(view.mask, view.K, view.R_wc, view.t_wc,
                                            num_particles, depth_range, rng, ray=ray,
                                            depth_hypothesis=depth_hypothesis,
                                            distribution=distribution,
                                            hypothesis_std=hypothesis_std))
    if not particles:
        raise ValueError("No valid views to triangulate")
    return triangulate(np.vstack(particles), rays, method=method), "particle_triangulation"
