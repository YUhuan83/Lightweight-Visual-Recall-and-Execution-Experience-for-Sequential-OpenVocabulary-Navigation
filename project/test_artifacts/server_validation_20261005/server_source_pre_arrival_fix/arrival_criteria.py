"""Shared, auditable target-arrival measurements."""
from __future__ import annotations

import math


# Match the execution bridge's action-success guard. A target observation does
# not count as an arrival while the vehicle is still several metres from its
# computed observation pose.
STANDOFF_ARRIVAL_TOLERANCE_M = 1.5
OBJECT_SURFACE_ARRIVAL_DISTANCE_M = 4.5
MASK_CONFIRMATION_MAX_SKEW_NS = 500_000_000


def planar_distance(first, second):
    if not isinstance(first, (list, tuple)) or not isinstance(second, (list, tuple)):
        return math.inf
    if len(first) < 2 or len(second) < 2:
        return math.inf
    try:
        values = tuple(float(value) for value in (first[0], first[1], second[0], second[1]))
    except (TypeError, ValueError):
        return math.inf
    if not all(math.isfinite(value) for value in values):
        return math.inf
    return math.hypot(values[0] - values[2], values[1] - values[3])


def memory_region_metrics(observed_position, observed_radius, remembered_view):
    """Return (distance, tolerance) for a fresh observation vs one memory view."""
    if not isinstance(remembered_view, dict):
        return math.inf, 0.0
    gap = planar_distance(observed_position, remembered_view.get("object_position"))
    if not math.isfinite(gap):
        return math.inf, 0.0
    try:
        seen_radius = float(observed_radius)
        remembered_radius = float(remembered_view.get("object_radius", 0.0))
    except (TypeError, ValueError):
        return math.inf, 0.0
    if not math.isfinite(seen_radius) or not math.isfinite(remembered_radius):
        return math.inf, 0.0
    tolerance = 3.0 + max(0.0, seen_radius) + max(0.0, remembered_radius)
    if remembered_view.get("evidence") == "passive_semantic_landmarks":
        tolerance = max(8.0, tolerance)
    return gap, tolerance


def mask_matches_confirmation(mask_query, mask_stamp_ns, confirmation,
                              max_skew_ns=MASK_CONFIRMATION_MAX_SKEW_NS):
    """Require the semantic mask and geometric confirmation to describe one frame."""
    if not isinstance(confirmation, dict) or confirmation.get("query") != mask_query:
        return False
    try:
        mask_stamp_ns = int(mask_stamp_ns)
        confirmation_stamp_ns = int(confirmation["observation_stamp_ns"])
        max_skew_ns = int(max_skew_ns)
    except (KeyError, TypeError, ValueError, OverflowError):
        return False
    return (mask_stamp_ns > 0 and confirmation_stamp_ns > 0 and max_skew_ns >= 0
            and abs(mask_stamp_ns - confirmation_stamp_ns) <= max_skew_ns)


def arrival_status_matches(status, query, confirmation,
                           tolerance_m=STANDOFF_ARRIVAL_TOLERANCE_M,
                           object_tolerance_m=OBJECT_SURFACE_ARRIVAL_DISTANCE_M):
    """Check that the exact observation was accepted near both goal and object."""
    if not isinstance(status, dict) or not isinstance(confirmation, dict):
        return False
    if status.get("status") != "object_found" or status.get("query") != query:
        return False
    status_confirmation = status.get("confirmation")
    if not isinstance(status_confirmation, dict):
        return False
    try:
        same_observation = (
            status_confirmation.get("query") == query
            and int(status_confirmation["observation_stamp_ns"])
            == int(confirmation["observation_stamp_ns"])
        )
        distance_m = float(status["standoff_distance_m"])
        reported_tolerance_m = float(status["standoff_tolerance_m"])
        object_distance_m = float(status["object_distance_m"])
        reported_object_tolerance_m = float(status["object_arrival_tolerance_m"])
        tolerance_m = float(tolerance_m)
        object_tolerance_m = float(object_tolerance_m)
    except (KeyError, TypeError, ValueError, OverflowError):
        return False
    return (same_observation and math.isfinite(distance_m)
            and math.isfinite(reported_tolerance_m) and math.isfinite(tolerance_m)
            and distance_m <= min(reported_tolerance_m, tolerance_m)
            and math.isfinite(object_distance_m)
            and math.isfinite(reported_object_tolerance_m)
            and math.isfinite(object_tolerance_m)
            and object_distance_m <= min(reported_object_tolerance_m, object_tolerance_m))
