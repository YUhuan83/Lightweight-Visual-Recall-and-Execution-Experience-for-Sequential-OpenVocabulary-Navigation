"""ROS-independent boundaries and completion checks for car maneuvers."""
from __future__ import annotations

import math


def motion_sections(points):
    """Return inclusive pose ranges sharing each forward/reverse cusp.

    Match Humble RPP's negative edge-dot-product cusp test. Ignore repeated
    positions when detecting a cusp; keep every original pose in the ranges.
    Each FollowPath goal must contain only one motion direction so nearest
    point pruning cannot jump to the branch after a cusp.
    """
    if len(points) < 2:
        return []
    boundaries = [0]
    previous = None
    for index, (a, b) in enumerate(zip(points, points[1:])):
        edge = (b[0] - a[0], b[1] - a[1])
        if math.hypot(*edge) <= 1e-9:
            continue
        if previous is not None and previous[0] * edge[0] + previous[1] * edge[1] < 0:
            boundaries.append(index)
        previous = edge
    boundaries.append(len(points) - 1)
    return list(zip(boundaries, boundaries[1:]))


def maneuver_reached(position, heading, target, target_heading,
                     position_tolerance=0.2, heading_tolerance=0.4):
    """Independently confirm the cusp before sending the next direction."""
    values = (*position, heading, *target, target_heading)
    if not all(math.isfinite(v) for v in values):
        return False
    heading_error = math.atan2(math.sin(heading - target_heading),
                               math.cos(heading - target_heading))
    return (math.hypot(position[0] - target[0], position[1] - target[1])
            <= position_tolerance and abs(heading_error) <= heading_tolerance)
