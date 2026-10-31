"""Pure helpers for goal de-duplication (no ROS dependency, offline testable)."""
from __future__ import annotations

import math


def should_replan(last_xy, new_xy, min_distance):
    """True when the new goal differs enough from the last planned goal."""
    if min_distance < 0:
        raise ValueError("min_distance must be >= 0")
    if new_xy is None:
        return False
    if last_xy is None:
        return True
    dx = float(new_xy[0]) - float(last_xy[0])
    dy = float(new_xy[1]) - float(last_xy[1])
    return math.hypot(dx, dy) >= float(min_distance)


def heading_changed(last_yaw, new_yaw, min_angle):
    """Detect a meaningful change in a curvature-constrained goal heading."""
    if min_angle < 0:
        raise ValueError("min_angle must be >= 0")
    if last_yaw is None or new_yaw is None:
        return False
    delta = float(new_yaw) - float(last_yaw)
    return abs(math.atan2(math.sin(delta), math.cos(delta))) >= min_angle


def should_retry_follow(status, retry_count, max_retries,
                        succeeded_status, canceled_status):
    """Retry failed FollowPath actions, but never canceled or successful ones."""
    if retry_count < 0 or max_retries < 0:
        raise ValueError("retry counts must be >= 0")
    return (status not in (succeeded_status, canceled_status)
            and retry_count < max_retries)


def update_direction_shift(current_gear, desired_gear, pending_gear,
                           pending_since, speed, now, shift_speed,
                           confirm_seconds):
    """Debounce a direction change and shift only after a low-speed dwell.

    Returns ``(gear, pending_gear, pending_since, should_brake)``. Requiring
    the opposite direction to remain requested prevents brief planner sign
    flips from repeatedly selecting drive and reverse.
    """
    if shift_speed < 0 or confirm_seconds < 0:
        raise ValueError("shift_speed and confirm_seconds must be >= 0")
    if desired_gear == current_gear:
        return current_gear, None, None, False
    if pending_gear != desired_gear or pending_since is None:
        return current_gear, desired_gear, float(now), True
    if float(now) - float(pending_since) >= confirm_seconds \
            and float(speed) <= shift_speed:
        return desired_gear, None, None, False
    return current_gear, desired_gear, pending_since, True
