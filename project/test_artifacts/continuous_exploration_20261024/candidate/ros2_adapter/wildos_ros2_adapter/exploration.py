"""Pure ExploRFM frontier/traversability direction selection."""
from __future__ import annotations

import math

import numpy as np


def exploration_bearing(frontier, traversability, fov_deg=91.5):
    """Select a traversable visual-frontier direction from dense head maps."""
    if frontier is None:
        return 0.0
    front = np.asarray(frontier, dtype=float)
    if front.ndim != 2 or not front.size or not np.isfinite(front).all():
        raise ValueError("frontier must be a finite non-empty 2D map")
    rows = slice(front.shape[0] // 3, None)
    score = front[rows].mean(axis=0)
    if traversability is not None:
        trav = np.asarray(traversability, dtype=float)
        if trav.shape != front.shape or not np.isfinite(trav).all():
            raise ValueError("traversability must match frontier")
        trav_score = trav[trav.shape[0] // 3:, :].mean(axis=0)
        score *= 0.25 + 0.75 * np.clip(trav_score, 0.0, 1.0)
    score = np.convolve(score, np.ones(9) / 9.0, mode="same")
    best = int(np.argmax(score))
    return math.radians((best / max(front.shape[1], 1) - 0.5) * fov_deg)


def smooth_bearing(previous, requested, max_step_deg=15.0):
    if previous is None:
        return float(requested)
    delta = (float(requested) - float(previous) + math.pi) % (2.0 * math.pi) - math.pi
    step = math.radians(max_step_deg)
    return float(previous) + float(np.clip(delta, -step, step))
