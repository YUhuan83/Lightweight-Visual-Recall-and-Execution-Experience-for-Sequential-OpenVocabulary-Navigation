"""ROS-independent boundaries and completion checks for car maneuvers."""
from __future__ import annotations

import math


# A pure-pursuit action can settle an Ackermann vehicle a little outside the
# cusp lattice point. Accept a modest endpoint disk, then independently check
# the actual pose before joining the next section or replanning from it.
MANEUVER_COMPLETION_POSITION_TOLERANCE_M = 0.6
# A reversing cusp is a trajectory junction, not a semantic observation region.
ACKERMANN_CUSP_POSITION_TOLERANCE_M = 0.2
NORMAL_GOAL_CHECKER_POSITION_TOLERANCE_M = 1.0


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


def first_section_away(points, target, away_threshold=2.0):
    """Report whether the first motion section moves away from ``target``.

    A continuous-start connector can open with a section that heads away from
    the goal before the searched suffix turns back.  trial_20261021_183339:
    the goal was ~25 m south, yet the dispatched first sections drove north
    (y=+0.2) for the first ~75 s.  This function only reports the geometry;
    callers stay responsible for whether to reject or accept the plan.
    """
    if away_threshold <= 0 or len(points) < 2 or len(target) != 2:
        return False
    sections = motion_sections(points)
    if not sections:
        return False
    first, last = sections[0]
    if last <= first:
        return False
    before = math.hypot(points[first][0] - target[0], points[first][1] - target[1])
    after = math.hypot(points[last][0] - target[0], points[last][1] - target[1])
    return after > before + away_threshold


def first_section_retreat_cost(points, target):
    """Return only the first section's increase in distance to a goal.

    This is a ranking hint for already-safe Ackermann alternatives. It never
    rejects a route and leaves necessary reversing available as a fallback.
    """
    if len(points) < 2 or len(target) != 2:
        return 0.0
    sections = motion_sections(points)
    if not sections:
        return 0.0
    first, last = sections[0]
    before = math.hypot(points[first][0] - target[0], points[first][1] - target[1])
    after = math.hypot(points[last][0] - target[0], points[last][1] - target[1])
    return max(0.0, after - before)


def maneuver_reached(position, heading, target, target_heading,
                     position_tolerance=0.3, heading_tolerance=0.4):
    """Independently confirm the cusp before sending the next direction."""
    values = (*position, heading, *target, target_heading)
    if not all(math.isfinite(v) for v in values):
        return False
    heading_error = math.atan2(math.sin(heading - target_heading),
                               math.cos(heading - target_heading))
    return (math.hypot(position[0] - target[0], position[1] - target[1])
            <= position_tolerance and abs(heading_error) <= heading_tolerance)


def diagnose_maneuver_join(points, headings, position, heading, minimum_turning_radius,
                           lookahead=3., *, endpoint_position_tolerance=MANEUVER_COMPLETION_POSITION_TOLERANCE_M):
    """Return the feasible cusp-join arc and a reason when it cannot be built."""
    if (len(points) < 2 or len(points) != len(headings) or len(position) != 2
            or any(len(point) != 2 for point in points)
            or minimum_turning_radius <= 0 or lookahead <= 0 or endpoint_position_tolerance <= 0
            or not all(math.isfinite(value) for value in
                       (*position, heading, minimum_turning_radius, lookahead,endpoint_position_tolerance))
            or any(not all(math.isfinite(value) for value in point) for point in points)
            or any(not math.isfinite(value) for value in headings)):
        return None, "invalid_join_input"
    position_error = math.hypot(position[0] - points[0][0], position[1] - points[0][1])
    heading_error = math.atan2(math.sin(heading - headings[0]),
                               math.cos(heading - headings[0]))
    if position_error > MANEUVER_COMPLETION_POSITION_TOLERANCE_M:
        return None, "cusp_position_outside_tolerance"
    if abs(heading_error) > .35:
        return None, "cusp_heading_outside_tolerance"
    direction = None
    target, target_heading = points[-1], headings[-1]
    target_is_endpoint = True
    for i, (a, b) in enumerate(zip(points, points[1:])):
        dx, dy = b[0]-a[0], b[1]-a[1]
        length2 = dx*dx+dy*dy
        if length2 < 1e-12:
            continue
        if direction is None:
            direction = 1 if dx*math.cos(headings[i])+dy*math.sin(headings[i]) >= 0 else -1
        ax, ay = a[0]-position[0], a[1]-position[1]
        linear = 2*(ax*dx+ay*dy)
        constant = ax*ax+ay*ay-lookahead*lookahead
        discriminant = linear*linear-4*length2*constant
        if discriminant < 0:
            continue
        t = (-linear+math.sqrt(discriminant))/(2*length2)
        if 0 <= t <= 1:
            target = (a[0]+t*dx, a[1]+t*dy)
            delta = math.atan2(math.sin(headings[i+1]-headings[i]),
                               math.cos(headings[i+1]-headings[i]))
            target_heading = headings[i]+t*delta
            target_is_endpoint = i == len(points)-2 and t >= 1.-1e-6
            break
    dx, dy = target[0]-position[0], target[1]-position[1]
    x = dx*math.cos(heading)+dy*math.sin(heading)
    y = -dx*math.sin(heading)+dy*math.cos(heading)
    if direction is None:
        return None, "path_has_no_motion_direction"
    if direction*x <= 0:
        return None, "join_target_behind_motion_direction"
    if x*x+y*y < 1e-8:
        return None, "join_target_at_current_pose"
    requested_curvature = 2*y/(x*x+y*y)
    requested_turn = 2*math.atan2(direction*y, direction*x)
    signed_length = (requested_turn/requested_curvature
                     if abs(requested_curvature) > 1e-8 else x)
    curvature = max(-1/minimum_turning_radius,
                    min(1/minimum_turning_radius, requested_curvature))
    theta = signed_length*curvature
    # Match the bounded Ackermann controller BEFORE collision prediction.
    # A lattice heading can request a tighter carrot circle than the actuator
    # supports. Predict its actual bounded arc, then require a close rejoin.
    lx = math.sin(theta)/curvature if abs(curvature) > 1e-8 else signed_length
    ly = (1-math.cos(theta))/curvature if abs(curvature) > 1e-8 else 0.
    end_gap = math.hypot(lx-x, ly-y)
    # An intermediate carrot can rejoin further along the path. A short
    # section ends here: its prediction must fit the actual checker disk.
    # Never relax the existing guard for a more permissive normal checker.
    gap_limit = (min(MANEUVER_COMPLETION_POSITION_TOLERANCE_M,endpoint_position_tolerance)
                 if target_is_endpoint else MANEUVER_COMPLETION_POSITION_TOLERANCE_M)
    if end_gap > gap_limit:
        return None, "join_curvature_exceeds_turning_limit"
    end_error = math.atan2(math.sin(heading+theta-target_heading),
                           math.cos(heading+theta-target_heading))
    if abs(end_error) > .35:
        return None, ("join_curvature_exceeds_turning_limit"
                      if abs(requested_curvature) > 1/minimum_turning_radius+1e-6
                      else "join_heading_misses_path")
    count = max(1, math.ceil(abs(signed_length)/.1))
    poses = []
    for step in range(count+1):
        s = signed_length*step/count
        turn = s*curvature
        lx = math.sin(turn)/curvature if abs(curvature) > 1e-8 else s
        ly = (1-math.cos(turn))/curvature if abs(curvature) > 1e-8 else 0.
        poses.append((position[0]+lx*math.cos(heading)-ly*math.sin(heading),
                      position[1]+lx*math.sin(heading)+ly*math.cos(heading), heading+turn))
    return poses, None


def maneuver_join(points, headings, position, heading, minimum_turning_radius,
                  lookahead=3.):
    """Build a bounded cusp-join arc; keep the original None/list API."""
    return diagnose_maneuver_join(points, headings, position, heading,
                                  minimum_turning_radius, lookahead)[0]


def initial_maneuver_already_reached(points, headings, position, heading,
                                     position_tolerance=.3):
    """Detect a cusp that SimpleGoalChecker would complete without driving.

    Lattice rounding can produce a short first reverse section inside the
    0.3 m arrival disk. Replanning from the unchanged axle repeats that section;
    reject this candidate so another heading/frontier can be considered.
    """
    sections = motion_sections(points)
    if len(sections) < 2 or len(points) != len(headings):
        return False
    cusp = sections[0][1]
    return maneuver_reached(position, heading, points[cusp], headings[cusp],
                            position_tolerance=position_tolerance, heading_tolerance=0.35)
