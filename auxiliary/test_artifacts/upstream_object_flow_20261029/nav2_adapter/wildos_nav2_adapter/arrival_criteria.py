"""Shared, auditable target-arrival measurements."""
from __future__ import annotations

import math
from .motion_path import MANEUVER_COMPLETION_POSITION_TOLERANCE_M
from .vehicle_geometry import SBR_FRONT_EXTENT, SBR_REAR_EXTENT, SBR_HALF_WIDTH


# Keep object confirmation aligned with the strict controller goal checker
# used for explicit semantic-target endpoints.
STANDOFF_ARRIVAL_TOLERANCE_M = MANEUVER_COMPLETION_POSITION_TOLERANCE_M
OBJECT_SURFACE_ARRIVAL_DISTANCE_M = 4.5
BODY_OBJECT_ARRIVAL_DISTANCE_M = 2.0
ARRIVAL_NUMERIC_EPSILON_M = 1e-9
MAX_CLAMPED_OBJECT_ARRIVAL_DISTANCE_M = 6.0
PASSIVE_SEMANTIC_MIN_ASSOCIATION_DISTANCE_M = 10.0
MASK_CONFIRMATION_MAX_SKEW_NS = 500_000_000


def vehicle_body_distance(vehicle_position, vehicle_heading, object_position, carrier_model):
    """Planar distance from the confirmed point to the carrier's outer footprint.

    The physical runtime contract already checks the SBR rear-axle polygon
    against these measured extents. Inflation padding is not body geometry.
    An ideal point carrier retains point-to-point distance.
    """
    reference_distance = planar_distance(vehicle_position, object_position)
    if (not math.isfinite(reference_distance)
            or not isinstance(vehicle_heading, (int, float))
            or not math.isfinite(vehicle_heading)):
        return math.inf
    if carrier_model == 'ideal_point':
        return reference_distance
    if carrier_model != 'ackermann':
        return math.inf
    dx = float(object_position[0])-float(vehicle_position[0])
    dy = float(object_position[1])-float(vehicle_position[1])
    c, s = math.cos(vehicle_heading), math.sin(vehicle_heading)
    x, y = c*dx+s*dy, -s*dx+c*dy
    outside_x = max(-SBR_REAR_EXTENT-x, x-SBR_FRONT_EXTENT, 0.)
    outside_y = max(abs(y)-SBR_HALF_WIDTH, 0.)
    return math.hypot(outside_x, outside_y)


def within_body_arrival(distance, limit=BODY_OBJECT_ARRIVAL_DISTANCE_M):
    return (math.isfinite(distance) and math.isfinite(limit) and 0 < limit <= BODY_OBJECT_ARRIVAL_DISTANCE_M
            and 0 <= distance <= limit+ARRIVAL_NUMERIC_EPSILON_M)


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


def effective_object_arrival_distance(configured_distance, standoff_clamped,
                                      standoff_object_distance):
    """Allow a small tolerance increase only for a map-clamped safe standoff.

    A nominal standoff may be moved farther from an obstacle by the local map.
    The target-distance bound then expands only by the controller's endpoint
    tolerance, using the triangle inequality. A fixed cap prevents an overly
    distant, map-clamped viewpoint from counting as arrival.
    """
    try:
        configured = float(configured_distance)
    except (TypeError, ValueError, OverflowError):
        return math.inf
    if not math.isfinite(configured) or configured <= 0:
        return math.inf
    if standoff_clamped is not True:
        return configured
    try:
        gap = float(standoff_object_distance)
    except (TypeError, ValueError, OverflowError):
        return configured
    if not math.isfinite(gap) or gap < 0:
        return configured
    return max(configured, min(MAX_CLAMPED_OBJECT_ARRIVAL_DISTANCE_M,
                               gap + STANDOFF_ARRIVAL_TOLERANCE_M))


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
        # Passive landmarks are sparse surface samples, not object centers.
        # The first fresh multi-frame confirmation can land on another facade
        # patch of the same large structure; latch that confirmed geometry as
        # the query-local anchor before applying the tighter radius again.
        tolerance = max(PASSIVE_SEMANTIC_MIN_ASSOCIATION_DISTANCE_M, tolerance)
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
    if status.get('arrival_basis') == 'upstream_goal_distance_with_body_clearance':
        # Navigation finishes by destination distance. Independently verify
        # that a fresh semantic observation also places that object nearby.
        try:
            position, heading = status['vehicle_position'], status['vehicle_heading']
            model = status['carrier_model']
            goal_distance = vehicle_body_distance(position, heading,
                status['object_position'], model)
            observed_distance = vehicle_body_distance(position, heading,
                confirmation['object_position'], model)
            limit = float(status['object_arrival_tolerance_m'])
            reported = float(status['object_distance_m'])
        except (KeyError, ValueError, TypeError, OverflowError):
            return False
        return (confirmation.get('query') == query and math.isfinite(reported)
                and abs(goal_distance-reported) <= 1e-3
                and within_body_arrival(goal_distance, limit)
                and within_body_arrival(observed_distance, limit)
                and planar_distance(status['object_position'],
                                    confirmation['object_position']) <= limit)
    status_confirmation = status.get("confirmation")
    if not isinstance(status_confirmation, dict):
        return False
    if status.get('arrival_basis') == 'vehicle_body_clearance':
        try:
            same = (status_confirmation.get('query') == query
                    and int(status_confirmation['observation_stamp_ns'])
                    == int(confirmation['observation_stamp_ns']))
            measured = vehicle_body_distance(status.get('vehicle_position'),
                status.get('vehicle_heading'), confirmation.get('object_position'),
                status.get('carrier_model'))
            reported = float(status['object_distance_m'])
            limit = float(status['object_arrival_tolerance_m'])
        except (KeyError, ValueError, TypeError, OverflowError):
            return False
        return (same and math.isfinite(reported) and abs(measured-reported) <= 1e-3
                and within_body_arrival(measured, limit))
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
    effective_object_tolerance_m = effective_object_arrival_distance(
        object_tolerance_m,
        status_confirmation.get("standoff_clamped"),
        status_confirmation.get("standoff_object_distance_m"))
    return (same_observation and math.isfinite(distance_m)
            and math.isfinite(reported_tolerance_m) and math.isfinite(tolerance_m)
            and reported_tolerance_m <= tolerance_m
            and distance_m <= reported_tolerance_m
            and math.isfinite(object_distance_m)
            and math.isfinite(reported_object_tolerance_m)
            and math.isfinite(effective_object_tolerance_m)
            and reported_object_tolerance_m <= effective_object_tolerance_m
            and object_distance_m <= min(reported_object_tolerance_m,
                                         effective_object_tolerance_m))


def matched_arrival_evidence(statuses, confirmations, masks, query,
                             query_started_ns, now_ns, now, minimum_mask_count):
    """Join bounded histories by observation identity, never by latest slots."""
    for item in reversed(statuses):
        if not 0 <= now-item.get('received_at', -math.inf) < 5.:
            continue
        status = item.get('status', {})
        for confirmation in reversed(confirmations):
            stamp = confirmation.get('observation_stamp_ns', 0)
            if (confirmation.get('query') != query
                    or confirmation.get('search_confirmed') is not True
                    or not isinstance(stamp, int) or stamp < query_started_ns
                    or not 0 <= now_ns-stamp <= 5_000_000_000
                    or not 0 <= now-confirmation.get('received_at', -math.inf) < 5.
                    or not arrival_status_matches(status, query, confirmation)):
                continue
            for mask in reversed(masks):
                if (mask.get('query') == query
                        and mask.get('count', 0) >= minimum_mask_count
                        and mask.get('pixels', 0) > 0
                        and mask.get('observation_stamp_ns', 0) >= query_started_ns
                        and 0 <= now-mask.get('at', -math.inf) < 5.
                        and mask_matches_confirmation(query, mask.get('observation_stamp_ns'), confirmation)):
                    return dict(item, confirmation=confirmation, mask=mask)
    return None
