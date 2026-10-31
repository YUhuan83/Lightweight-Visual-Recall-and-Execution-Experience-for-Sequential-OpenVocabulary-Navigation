"""Use a visible target's bearing to rank existing exploration frontiers.

This supplies direction only. It never estimates object range, creates a goal,
changes traversability, or confirms an object search.
"""
import math
import numpy as np


def mask_bearing(mask, intrinsic, world_rotation):
    mask = np.asarray(mask)
    k, rotation = np.asarray(intrinsic), np.asarray(world_rotation)
    if (mask.ndim != 2 or k.shape != (3, 3) or rotation.shape != (3, 3)
            or not np.all(np.isfinite(k)) or not np.all(np.isfinite(rotation))
            or k[0, 0] <= 0 or k[1, 1] <= 0):
        return None
    pixels = np.argwhere(mask > 0)
    if not len(pixels):
        return None
    v, u = np.median(pixels, axis=0)
    try:
        ray = rotation @ np.linalg.solve(k, [u, v, 1.])
    except np.linalg.LinAlgError:
        return None
    if not np.all(np.isfinite(ray)) or np.linalg.norm(ray[:2]) < .1:
        return None
    return math.atan2(ray[1], ray[0])


def guide_frontier_scores(scores, observer_xy, frontier_positions, bearing,
                          maximum=.9, cone=math.pi / 6, maximum_distance=20.):
    scores = np.asarray(scores, dtype=np.float32).copy()
    if bearing is None or not math.isfinite(bearing) or scores.ndim != 2:
        return scores
    positions = np.asarray(frontier_positions, dtype=float)
    if positions.shape != (scores.shape[0], 2) or not scores.shape[1]:
        return scores
    offsets = positions - np.asarray(observer_xy, dtype=float)
    headings = np.arctan2(offsets[:, 1], offsets[:, 0])
    delta = np.arctan2(np.sin(headings - bearing), np.cos(headings - bearing))
    distances = np.linalg.norm(offsets, axis=1)
    eligible = (np.all(np.isfinite(offsets), axis=1)
                & (distances > .2) & (distances <= maximum_distance)
                & (np.abs(delta) <= cone))
    bins = np.arange(scores.shape[1]) * (2 * math.pi / scores.shape[1])
    bin_delta = np.arctan2(np.sin(bins - bearing), np.cos(bins - bearing))
    profile = maximum * np.exp(-.5 * (bin_delta / cone) ** 2)
    for index in np.flatnonzero(eligible):
        # Retain zero/invalid scores: a bearing cannot reopen an unsupported
        # direction. Geometry, physical feasibility and route validation still
        # decide whether a selected frontier can actually be approached.
        # The graph uses 1e-4 as its numerical floor for rejected directions.
        supported = np.isfinite(scores[index]) & (scores[index] > 1e-4)
        scores[index, supported] = np.maximum(scores[index, supported], profile[supported])
    return scores
