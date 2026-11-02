"""Small task model and geometry; independent of ROS and actuator transport."""

from dataclasses import dataclass
from enum import Enum, auto
import math


class State(Enum):
    IDLE = auto()
    EXPLORING = auto()
    APPROACHING_TARGET = auto()
    RECOVERING = auto()
    ARRIVED = auto()


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def arrived(position, target, evidence, query, now_ns):
    if target is None or not evidence or evidence.get("query") != query:
        return False
    stamp = evidence.get("observation_stamp_ns", 0)
    observed = evidence.get("object_position", ())
    return (
        evidence.get("search_confirmed") is True
        and len(observed) == 3
        and all(math.isfinite(x) for x in (*position, *target, *observed))
        # Freshness is checked when accepting the observation. The confirmed
        # object belongs to this task until query change, even if its surface
        # leaves the camera's view while the vehicle approaches.
        and 0 < stamp <= now_ns
        and distance(target, observed) < 3.0
        and distance(position, target) < 3.0
        and distance(position, observed) < 3.0
    )


@dataclass
class Task:
    query: str = ""
    state: State = State.IDLE
    target: tuple | None = None

    def begin(self, query):
        self.query = " ".join(query.casefold().split())
        self.target = None
        self.state = State.EXPLORING if self.query else State.IDLE

    def refine(self, point):
        if not self.query or self.state is State.ARRIVED:
            return False
        if len(point) != 3 or not all(math.isfinite(v) for v in point):
            return False
        self.target = tuple(point)
        self.state = State.APPROACHING_TARGET
        return True


def route_candidates(points, position, heading, target=None):
    """Bounded route samples, with local alternatives for an explicit goal.

    These are requests to Smac, not paths or permissions to drive. Smac owns
    full-body collision and curvature validation.
    """
    candidates = []
    terminal = []
    if points:
        # Preserve v74's projection and arclength sampling. Graph vertices are
        # topology, not the only rear-axle poses a vehicle may plan towards.
        best = (math.inf, 0, points[0])
        for index, (a, b) in enumerate(zip(points, points[1:])):
            dx, dy = b[0] - a[0], b[1] - a[1]
            length2 = dx * dx + dy * dy
            fraction = (
                0.0
                if length2 < 1e-9
                else max(
                    0.0,
                    min(
                        1.0,
                        ((position[0] - a[0]) * dx + (position[1] - a[1]) * dy)
                        / length2,
                    ),
                )
            )
            projection = (a[0] + fraction * dx, a[1] + fraction * dy)
            gap = distance(position, projection)
            if gap < best[0] - 1e-6:
                best = (gap, index, projection)
        remaining = [best[2], *points[best[1] + 1 :]]
        route = [position]
        for point in remaining:
            if distance(route[-1], point) > 1e-6:
                route.append(point)
        length, next_sample = 0.0, 4.0
        for a, b in zip(route, route[1:]):
            segment = distance(a, b)
            heading_to_end = math.atan2(b[1] - a[1], b[0] - a[0])
            while next_sample <= min(24.0, length + segment) + 1e-6:
                fraction = (next_sample - length) / segment
                candidates.append(
                    (
                        a[0] + fraction * (b[0] - a[0]),
                        a[1] + fraction * (b[1] - a[1]),
                        heading_to_end,
                    )
                )
                next_sample += 2.0
            length += segment
            if length >= 24.0:
                break
        total = sum(distance(a, b) for a, b in zip(route, route[1:]))
        if len(route) > 1 and total <= 24.0 and distance(position, route[-1]) > 1.0:
            a, b = route[-2:]
            if not candidates or distance(candidates[-1], b) > 1e-6:
                candidates.append((*b[:2], math.atan2(b[1] - a[1], b[0] - a[0])))
        candidates.reverse()
    if target is not None:
        gap = distance(position, target)
        bearing = math.atan2(target[1] - position[1], target[0] - position[0])
        forward = []
        if gap > 6.0:
            step = min(8.0, gap - 2.0)
            for lateral in (0.0, 2.0, -2.0):
                forward.append(
                    (
                        position[0]
                        + step * math.cos(bearing)
                        - lateral * math.sin(bearing),
                        position[1]
                        + step * math.sin(bearing)
                        + lateral * math.cos(bearing),
                        bearing,
                    )
                )
        if gap <= 24.0:
            # Search feasible rear-axle poses INSIDE the arrival disk. A wall
            # point itself need not be a collision-free vehicle destination.
            for offset in (0.0, math.pi / 2, -math.pi / 2, math.pi):
                angle = bearing + offset
                point = (
                    target[0] - 2.0 * math.cos(angle),
                    target[1] - 2.0 * math.sin(angle),
                )
                for orientation in (angle + math.pi / 2, angle - math.pi / 2):
                    terminal.append((*point, orientation))
        if gap <= 6.0:
            forward = terminal
        if not candidates or gap <= 6.0 or distance(candidates[0], target) >= gap:
            candidates = forward + candidates
        else:
            candidates += forward
    unique = []
    for candidate in candidates:
        if distance(position, candidate) > 0.5 and not any(
            distance(candidate, v) < 1e-6
            and abs(math.remainder(candidate[2] - v[2], 2 * math.pi)) < 1e-6
            for v in unique
        ):
            unique.append(candidate)
    unique = unique[:10]
    if target is not None and distance(position, target) > 6.0:
        # A target bearing is not the car's heading. Preserve the existing
        # requests, but also let Smac approach along the current road before
        # turning towards an off-road object. Reserve these three probes
        # after the preferred route candidate so long routes cannot hide them.
        gap = distance(position, target)
        step = min(8.0, gap - 2.0)
        vehicle_forward = []
        for lateral in (0.0, 2.0, -2.0):
            candidate = (
                position[0] + step * math.cos(heading) - lateral * math.sin(heading),
                position[1] + step * math.sin(heading) + lateral * math.cos(heading),
                heading,
            )
            if distance(candidate, target) < gap and not any(
                distance(candidate, other) < 1e-6
                and abs(math.remainder(candidate[2] - other[2], 2 * math.pi)) < 1e-6
                for other in unique[:1]
            ):
                vehicle_forward.append(candidate)
        unique = (
            unique[:1]
            + vehicle_forward
            + [
                candidate
                for candidate in unique[1:]
                if not any(
                    distance(candidate, other) < 1e-6
                    and abs(math.remainder(candidate[2] - other[2], 2 * math.pi)) < 1e-6
                    for other in vehicle_forward
                )
            ]
        )
    if terminal and gap > 6.0:
        # A valid arrival pose must be requested while it is inside the
        # planning horizon, not only after getting within6m. Preserve the
        # earlier requests as fallbacks; Smac still decides reachability.
        unique = terminal + [
            candidate
            for candidate in unique
            if not any(
                distance(candidate, other) < 1e-6
                and abs(math.remainder(candidate[2] - other[2], 2 * math.pi)) < 1e-6
                for other in terminal
            )
        ]
    return unique


class RecallCandidatePolicy:
    """Compare observed views with explicit geometry; no Node state mutation."""

    @staticmethod
    def rank(views, position, heading):
        def cost(view):
            point = view["position"]
            turn = abs(
                math.remainder(
                    math.atan2(point[1] - position[1], point[0] - position[0])
                    - heading,
                    2 * math.pi,
                )
            )
            confidence = min(1.0, max(0.0, float(view.get("confidence", 1.0))))
            return distance(position, point) + 2.0 * turn + 3.0 * (1.0 - confidence)

        valid = [
            v
            for v in views
            if len(v.get("position", ())) == 3
            and all(math.isfinite(x) for x in v["position"])
        ]
        return sorted(valid, key=cost)
