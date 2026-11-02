"""Scoring helpers for semantic object search."""

import math

import numpy as np


def bias_frontiers_on_traversable_mask(
    frontier_scores,
    traversability,
    object_mask,
    object_frontier_score,
    traversability_threshold,
):
    """Bias exploration toward masked targets without changing traversability.

    A semantic mask says what a pixel may depict; it does not show that the
    ground beneath it is drivable. Only apply target attraction where the
    model already considers the pixel traversable. The caller must pass the
    original traversability prediction unchanged to reachability scoring.
    """
    frontiers = np.asarray(frontier_scores, dtype=np.float32)
    travel = np.asarray(traversability, dtype=np.float32)
    mask = np.asarray(object_mask)
    if frontiers.shape != travel.shape or frontiers.shape != mask.shape:
        raise ValueError("frontier, traversability, and object-mask shapes must match")
    if (
        not math.isfinite(object_frontier_score)
        or not 0.0 <= object_frontier_score <= 1.0
    ):
        raise ValueError("object_frontier_score must be finite and in [0, 1]")
    if (
        not math.isfinite(traversability_threshold)
        or not 0.0 <= traversability_threshold <= 1.0
    ):
        raise ValueError("traversability_threshold must be finite and in [0, 1]")

    target_on_drivable_pixels = (
        np.isfinite(mask)
        & (mask > 0)
        & np.isfinite(travel)
        & (travel >= traversability_threshold)
    )
    target_bias = target_on_drivable_pixels.astype(np.float32) * object_frontier_score
    return np.maximum(frontiers, target_bias)
