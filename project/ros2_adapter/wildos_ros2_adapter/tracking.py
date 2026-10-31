"""Pure multi-frame semantic target confirmation."""
from __future__ import annotations

import numpy as np


def update_track(track, candidate, max_jump=8.0, confirmations=3,
                 miss_tolerance=2, confirmed_miss_tolerance=10):
    if candidate is None:
        track["misses"] = track.get("misses", 0) + 1
        if (track.get("confirmed") is not None
                and track["misses"] > confirmed_miss_tolerance):
            track["confirmed"] = None
            track["observations"] = []
            track["count"] = 0
            return None
        if track["misses"] > miss_tolerance and track.get("confirmed") is None:
            track["observations"] = []
            track["count"] = 0
        return track.get("confirmed")
    candidate = np.asarray(candidate, dtype=float)[:3]
    track["misses"] = 0
    if track.get("confirmed") is not None:
        return track["confirmed"]
    observations = [np.asarray(v, dtype=float)[:3] for v in track.get("observations", [])]
    if observations:
        center = np.median(np.asarray(observations), axis=0)
        if float(np.linalg.norm(candidate[:2] - center[:2])) > max_jump:
            observations = []
    observations.append(candidate)
    track["observations"] = observations[-confirmations:]
    track["count"] = len(track["observations"])
    if track["count"] >= confirmations:
        center = np.median(np.asarray(track["observations"]), axis=0)
        track["confirmed"] = [float(v) for v in center]
    return track.get("confirmed")
