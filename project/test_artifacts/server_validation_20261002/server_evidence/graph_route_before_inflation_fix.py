"""Pure geometry and costmap checks for GraphNav-guided vehicle planning."""
from __future__ import annotations

import math


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def freshness_issue(now, odom_received_at, costmap_received_at,
                   odom_timeout, costmap_timeout):
    """Return which planning input is missing or older than its timeout."""
    if not math.isfinite(now):
        return "stale_odom", None
    for name, received_at, timeout in (
            ("odom", odom_received_at, odom_timeout),
            ("costmap", costmap_received_at, costmap_timeout)):
        if (received_at is None or not math.isfinite(received_at)
                or not math.isfinite(timeout) or timeout <= 0):
            return "stale_" + name, None
        age = now - received_at
        if age < 0 or age > timeout:
            return "stale_" + name, max(0.0, age)
    return None


def cell_cost(grid, x, y):
    info = grid.info
    if info.resolution <= 0:
        return 100
    q = info.origin.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))
    dx, dy = x - info.origin.position.x, y - info.origin.position.y
    mx = math.floor((math.cos(yaw) * dx + math.sin(yaw) * dy) / info.resolution)
    my = math.floor((-math.sin(yaw) * dx + math.cos(yaw) * dy) / info.resolution)
    if not (0 <= mx < info.width and 0 <= my < info.height):
        return 100
    return grid.data[my * info.width + mx]


def footprint_blockers(grid, center, heading, *, front=2.3, rear=2.3,
                       half_width=1.0, limit=5):
    """Return occupied cells under the configured rectangular vehicle body."""
    step = min(max(grid.info.resolution / 2, 0.1), 0.25)
    cos_yaw, sin_yaw = math.cos(heading), math.sin(heading)
    blocked = []
    longitudinal = math.ceil((front + rear) / step)
    lateral = math.ceil(2 * half_width / step)
    for i in range(longitudinal + 1):
        forward = -rear + (front + rear) * i / longitudinal
        for j in range(lateral + 1):
            left = -half_width + 2 * half_width * j / lateral
            x = center[0] + cos_yaw * forward - sin_yaw * left
            y = center[1] + sin_yaw * forward + cos_yaw * left
            if cell_cost(grid, x, y) >= 99:
                blocked.append((round(forward, 2), round(left, 2)))
                if len(blocked) >= limit:
                    return blocked
    return blocked


def remaining_route(start, route):
    """Project onto the ordered route and discard its already traversed prefix."""
    if not route:
        return []
    if len(route) == 1:
        return list(route)
    best = (math.inf, 0, route[0])
    for index, (a, b) in enumerate(zip(route, route[1:])):
        vx, vy = b[0] - a[0], b[1] - a[1]
        length2 = vx * vx + vy * vy
        t = 0.0 if length2 < 1e-9 else max(0.0, min(1.0,
            ((start[0] - a[0]) * vx + (start[1] - a[1]) * vy) / length2))
        projection = (a[0] + t * vx, a[1] + t * vy)
        gap = distance(start, projection)
        if gap < best[0] - 1e-6:
            best = (gap, index, projection)
    return [best[2]] + list(route[best[1] + 1:])


def route_candidates(start, route, grid, *, minimum=4.0, maximum=24.0,
                     spacing=2.0, limit=10):
    """Return farthest free route samples first, with tangent headings."""
    if len(route) < 1 or grid is None:
        return []
    points = [start]
    for point in remaining_route(start, route):
        if distance(points[-1], point) > 1e-6:
            points.append(point)
    if len(points) < 2:
        return []
    total_length = sum(distance(a, b) for a, b in zip(points, points[1:]))
    samples = []
    arclength = 0.0
    next_sample = minimum
    for a, b in zip(points, points[1:]):
        segment = distance(a, b)
        if segment < 1e-6:
            continue
        while next_sample <= min(maximum, arclength + segment) + 1e-6:
            fraction = (next_sample - arclength) / segment
            x = a[0] + fraction * (b[0] - a[0])
            y = a[1] + fraction * (b[1] - a[1])
            cost = cell_cost(grid, x, y)
            if 0 <= cost < 99:
                samples.append((x, y, math.atan2(b[1] - a[1], b[0] - a[0]), next_sample))
            next_sample += spacing
        arclength += segment
        if arclength >= maximum:
            break
    # Include the exact endpoint, including the last 1–4 m of a segment.
    if (total_length <= maximum and distance(start, points[-1]) > 1.0
            and not any(distance(sample[:2], points[-1]) < 1e-6 for sample in samples)):
        a, b = points[-2], points[-1]
        cost = cell_cost(grid, *b)
        if 0 <= cost < 99:
            samples.append((b[0], b[1], math.atan2(b[1] - a[1], b[0] - a[0]),
                            total_length))
    return list(reversed(samples))[:limit]


def distance_to_route(point, route):
    if not route:
        return math.inf
    if len(route) == 1:
        return distance(point, route[0])
    best = math.inf
    for a, b in zip(route, route[1:]):
        vx, vy = b[0] - a[0], b[1] - a[1]
        length2 = vx * vx + vy * vy
        t = 0.0 if length2 < 1e-9 else max(0.0, min(1.0,
            ((point[0] - a[0]) * vx + (point[1] - a[1]) * vy) / length2))
        best = min(best, distance(point, (a[0] + t * vx, a[1] + t * vy)))
    return best


def validate_vehicle_path(points, headings, route, grid, target,
                          *, corridor=9.0, max_unknown_fraction=0.20,
                          minimum_turning_radius=8.0, start=None,
                          target_heading=None, heading_quantization_bins=0):
    """Reject empty, obstructed, distant, or implausibly sharp Nav2 paths."""
    if not math.isfinite(minimum_turning_radius) or minimum_turning_radius <= 0:
        return "invalid_minimum_turning_radius"
    if len(points) < 2 or len(points) != len(headings):
        return "empty_path"
    if (any(not all(map(math.isfinite, p)) for p in points)
            or any(not math.isfinite(h) for h in headings)
            or any(not all(map(math.isfinite, p)) for p in route)
            or not all(map(math.isfinite, target))):
        return "nonfinite_path"
    if distance(points[-1], target) > 1.5:
        return "wrong_endpoint"
    if start is not None and distance(points[0], start) > 1.0:
        return "wrong_start"
    if target_heading is not None:
        if not math.isfinite(target_heading):
            return "nonfinite_path"
        error = math.atan2(math.sin(headings[-1] - target_heading),
                           math.cos(headings[-1] - target_heading))
        if abs(error) > 0.7:
            return "wrong_heading"
    sampled = [points[0]]
    sampled_headings = [headings[0]]
    traveled = [0.0]
    for a, b in zip(points, points[1:]):
        length = distance(a, b)
        steps = max(1, math.ceil(length / max(grid.info.resolution, 0.1)))
        heading_index = len(traveled) - 1
        heading_delta = math.atan2(
            math.sin(headings[heading_index + 1] - headings[heading_index]),
            math.cos(headings[heading_index + 1] - headings[heading_index]))
        for step in range(1, steps + 1):
            t = step / steps
            sampled.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
            sampled_headings.append(headings[heading_index] + t * heading_delta)
        # Nav2 samples circular motion primitives as chords. Heading change
        # divided by chord length slightly overestimates their true curvature
        # (the live 8 m primitive appeared as 7.997 m). Recover arc length
        # from the chord and its heading change before checking steering.
        turn = abs(heading_delta)
        arc_length = (length * turn / (2 * math.sin(turn / 2))
                      if turn > 1e-6 else length)
        traveled.append(traveled[-1] + arc_length)
    unknown = 0
    for p in sampled:
        cost = cell_cost(grid, *p)
        if cost >= 99:
            return "collision_cell"
        unknown += cost < 0
        if distance_to_route(p, route) > corridor:
            return "outside_graph_corridor"
    if unknown / len(sampled) > max_unknown_fraction:
        return "unknown_costmap"
    for point, heading in zip(sampled, sampled_headings):
        if footprint_blockers(grid, point, heading, limit=1):
            return "footprint_collision"
    # Reeds-Shepp lattice poses can repeat a quantized heading across two
    # half-primitives. A window ending halfway through a primitive must not
    # count its full heading-bin change as continuous curvature. Bound that
    # uncertainty and independently check the actual sampled geometry below.
    bin_angle = 2 * math.pi / heading_quantization_bins if heading_quantization_bins > 0 else 0.
    quantized = bool(bin_angle) and all(
        abs(h / bin_angle - round(h / bin_angle)) < 1e-4 for h in headings)
    # Check heading changes over travelled distance, ignoring zero-length poses.
    for i in range(len(points) - 1):
        j = i + 1
        while j < len(points) - 1 and traveled[j] - traveled[i] < 2.0:
            j += 1
        arc = traveled[j] - traveled[i]
        if arc < 0.5:
            continue
        turn = math.atan2(math.sin(headings[j] - headings[i]),
                          math.cos(headings[j] - headings[i]))
        bounded_turn = max(0., abs(turn) - bin_angle) if quantized else abs(turn)
        if bounded_turn / arc > 1.0 / minimum_turning_radius + 1e-6:
            return "excess_curvature"
    if quantized:
        # Three points separated by at least one metre recover a circular
        # primitive's radius without using its discretized pose heading.
        # Split at forward/reverse cusps: a direction change is not a body turn.
        directions = [1 if (b[0]-a[0])*math.cos(h)+(b[1]-a[1])*math.sin(h) >= 0 else -1
                      for a, b, h in zip(points, points[1:], headings)]
        for i in range(len(points)-2):
            j = i+1
            while j < len(points)-1 and distance(points[i], points[j]) < 1.:
                j += 1
            k = j+1
            while k < len(points)-1 and distance(points[j], points[k]) < 1.:
                k += 1
            if k >= len(points) or distance(points[j], points[k]) < 1.:
                continue
            if len(set(directions[i:k])) > 1:
                continue
            a, b, c = points[i], points[j], points[k]
            product = distance(a,b)*distance(b,c)*distance(c,a)
            if product < 1e-9:
                continue
            curvature = 2*abs((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]))/product
            # Nav2's float32 path positions lose a few micrometres at Town's
            # world coordinates. This tolerance is below 0.02% at radius 8 m.
            if curvature > 1./minimum_turning_radius + 2e-5:
                return "excess_curvature"
    return None
