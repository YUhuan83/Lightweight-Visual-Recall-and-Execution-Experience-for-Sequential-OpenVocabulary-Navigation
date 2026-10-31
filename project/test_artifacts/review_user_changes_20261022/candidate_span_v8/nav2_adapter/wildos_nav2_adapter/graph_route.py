"""Pure geometry and costmap checks for GraphNav-guided vehicle planning."""
from __future__ import annotations

import math
from .vehicle_geometry import SBR_FRONT_EXTENT, SBR_REAR_EXTENT, SBR_HALF_WIDTH


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def maneuver_cost(points):
    """Length plus a 3 m equivalent cost per gear change."""
    from .motion_path import motion_sections
    return (sum(distance(a, b) for a, b in zip(points, points[1:]))
            + 3.0*max(0, len(motion_sections(points))-1))


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


def raster_line(a, b):
    """Nav2 LineIterator cells, including its integer tie handling."""
    x, y = a
    tx, ty = b
    dx, dy = abs(tx-x), abs(ty-y)
    xi1 = xi2 = 1 if tx >= x else -1
    yi1 = yi2 = 1 if ty >= y else -1
    if dx >= dy:
        xi1, yi2, den, num, add, count = 0, 0, dx, dx//2, dy, dx
    else:
        xi2, yi1, den, num, add, count = 0, 0, dy, dy//2, dx, dy
    for _ in range(count+1):
        yield x, y
        num += add
        if den and num >= den:
            num -= den
            x, y = x+xi1, y+yi1
        x, y = x+xi2, y+yi2


def footprint_blockers(grid, center, heading, *, front=SBR_FRONT_EXTENT,
                       rear=SBR_REAR_EXTENT, half_width=SBR_HALF_WIDTH,
                       padding=0.01, limit=5):
    """Return lethal cells under the configured rectangular vehicle body.

    Nav2 publishes inscribed inflation as 99 and actual obstacles as 100.
    Testing every body sample against 99 would apply the inscribed radius
    again around the whole footprint. Path centres still reject 99 below.
    """
    # Match both Nav2 costmaps' explicit footprint_padding. Rasterize edges
    # exactly like the controller, then check interior cell centres as well.
    # Sparse body samples can miss a lethal edge cell on a rotated footprint.
    if (grid.info.resolution <= 0 or padding < 0
            or not all(map(math.isfinite, (*center, heading, padding)))):
        return [(0.0, 0.0)]
    front, rear, half_width = front+padding, rear+padding, half_width+padding
    cos_yaw, sin_yaw = math.cos(heading), math.sin(heading)
    blocked = []
    info = grid.info
    q = info.origin.orientation
    grid_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    cg, sg = math.cos(grid_yaw), math.sin(grid_yaw)
    corners = []
    for forward, left in ((front, half_width), (front, -half_width),
                          (-rear, -half_width), (-rear, half_width)):
        dx = center[0]+cos_yaw*forward-sin_yaw*left-info.origin.position.x
        dy = center[1]+sin_yaw*forward+cos_yaw*left-info.origin.position.y
        cell = (math.floor((cg*dx+sg*dy)/info.resolution),
                math.floor((-sg*dx+cg*dy)/info.resolution))
        if not (0 <= cell[0] < info.width and 0 <= cell[1] < info.height):
            return [(round(forward, 2), round(left, 2))]
        corners.append(cell)
    cells = set()
    for a, b in zip(corners, corners[1:]+corners[:1]):
        cells.update(raster_line(a, b))
    for my in range(min(p[1] for p in corners), max(p[1] for p in corners)+1):
        for mx in range(min(p[0] for p in corners), max(p[0] for p in corners)+1):
            gx, gy = (mx+.5)*info.resolution, (my+.5)*info.resolution
            dx = info.origin.position.x+cg*gx-sg*gy-center[0]
            dy = info.origin.position.y+sg*gx+cg*gy-center[1]
            forward, left = cos_yaw*dx+sin_yaw*dy, -sin_yaw*dx+cos_yaw*dy
            if -rear <= forward <= front and abs(left) <= half_width:
                cells.add((mx, my))
    for mx, my in sorted(cells):
        if grid.data[my*info.width+mx] >= 100:
            gx, gy = (mx+.5)*info.resolution, (my+.5)*info.resolution
            dx = info.origin.position.x+cg*gx-sg*gy-center[0]
            dy = info.origin.position.y+sg*gx+cg*gy-center[1]
            blocked.append((round(cos_yaw*dx+sin_yaw*dy, 2),
                            round(-sin_yaw*dx+cos_yaw*dy, 2)))
            if len(blocked) >= limit:
                return blocked
    return blocked


def point_blockers(grid, center, *, lethal_cost=99):
    """Check a zero-footprint carrier's map cell, without vehicle geometry."""
    if (grid is None or grid.info.resolution <= 0
            or not all(map(math.isfinite, center))):
        return [(0.0, 0.0)]
    if cell_cost(grid, *center) >= lethal_cost:
        return [(0.0, 0.0)]
    return []


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
            if cost < 99:
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
        if cost < 99:
            samples.append((b[0], b[1], math.atan2(b[1] - a[1], b[0] - a[0]),
                            total_length))
    return list(reversed(samples))[:limit]


def spread_route_endpoints(candidates):
    """Try both long and short prefixes within a bounded failure budget.

    Keep the heading alternatives at each endpoint together for the existing
    cost comparison. A few failures at neighbouring distant samples do not
    establish that the short, immediately executable prefix is unreachable.
    """
    groups = {}
    for candidate in candidates:
        groups.setdefault(candidate[:2], []).append(candidate)
    ordered = sorted(groups.values(), key=lambda group: group[0][3], reverse=True)
    output = []
    while ordered:
        output.extend(ordered.pop(0))
        if ordered:
            output.extend(ordered.pop())
    return output


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


def validate_local_prefix(points, headings, grid, start, start_heading, maximum=2.0):
    """Check the imminent same-direction sweep on the controller's own map.

    A local rolling map cannot validate the distant full route. Inspect the
    current body and first 2 m with the same padding/raster checks instead.
    """
    if footprint_blockers(grid, start, start_heading, limit=1):
        return 'local_start_footprint_blocked'
    travelled = 0.0
    for i, (a, b) in enumerate(zip(points, points[1:])):
        length = distance(a, b)
        if length <= 1e-9:
            continue
        checked = min(length, maximum-travelled)
        steps = max(1, math.ceil(checked / max(.05, grid.info.resolution/2)))
        delta = math.atan2(math.sin(headings[i+1]-headings[i]),
                           math.cos(headings[i+1]-headings[i]))
        for j in range(steps+1):
            fraction = checked/length*j/steps
            p = (a[0]+fraction*(b[0]-a[0]), a[1]+fraction*(b[1]-a[1]))
            h = headings[i]+fraction*delta
            cost = cell_cost(grid, *p)
            if cost < 0:
                return 'local_unknown_costmap'
            if cost >= 99:
                return 'local_collision_cell'
            if footprint_blockers(grid, p, h, limit=1):
                return 'local_footprint_collision'
        travelled += checked
        if travelled >= maximum:
            break
    return None


def validate_ideal_local_prefix(points, grid, start, maximum=2.0):
    """Check the imminent path for a point carrier; heading/turning are free."""
    if grid is None or point_blockers(grid, start):
        return 'local_start_blocked'
    travelled = 0.0
    for a, b in zip(points, points[1:]):
        length = distance(a, b)
        if length <= 1e-9:
            continue
        checked = min(length, maximum-travelled)
        steps = max(1, math.ceil(checked / max(.05, grid.info.resolution/2)))
        for j in range(steps+1):
            fraction = checked/length*j/steps
            p = (a[0]+fraction*(b[0]-a[0]), a[1]+fraction*(b[1]-a[1]))
            cost = cell_cost(grid, *p)
            if cost < 0:
                return 'local_unknown_costmap'
            if cost >= 99:
                return 'local_collision_cell'
        travelled += checked
        if travelled >= maximum:
            break
    return None


def validate_ideal_path(points, headings, route, grid, target,
                        *, corridor=8.0, max_unknown_fraction=1.0,
                        start=None, target_heading=None):
    """Validate a planar point-carrier route without curvature/footprint rules."""
    if grid is None or grid.info.resolution <= 0:
        return 'invalid_costmap'
    if len(points) < 2 or len(points) != len(headings):
        return 'empty_path'
    if (any(not all(map(math.isfinite, p)) for p in points)
            or any(not math.isfinite(h) for h in headings)
            or any(not all(map(math.isfinite, p)) for p in route)
            or not all(map(math.isfinite, target))):
        return 'nonfinite_path'
    if distance(points[-1], target) > 1.5:
        return 'wrong_endpoint'
    if start is not None and distance(points[0], start) > 1.0:
        return 'wrong_start'
    if target_heading is not None:
        if not math.isfinite(target_heading):
            return 'nonfinite_path'
        error = math.atan2(math.sin(headings[-1]-target_heading),
                           math.cos(headings[-1]-target_heading))
        if abs(error) > .7:
            return 'wrong_heading'
    sampled = [points[0]]
    for a, b in zip(points, points[1:]):
        length = distance(a, b)
        steps = max(1, math.ceil(length / max(grid.info.resolution, .1)))
        sampled.extend((a[0]+(b[0]-a[0])*step/steps,
                        a[1]+(b[1]-a[1])*step/steps)
                       for step in range(1, steps+1))
    unknown = 0
    for p in sampled:
        cost = cell_cost(grid, *p)
        if cost >= 99:
            return 'collision_cell'
        unknown += cost < 0
        if distance_to_route(p, route) > corridor:
            return 'outside_graph_corridor'
    if unknown / len(sampled) > max_unknown_fraction:
        return 'unknown_costmap'
    return None


def validate_vehicle_path(points, headings, route, grid, target,
                          *, corridor=None, max_unknown_fraction=1.0,
                          minimum_turning_radius=8.0, start=None,
                          target_heading=None, heading_quantization_bins=0):
    """Reject empty, obstructed, distant, or implausibly sharp Nav2 paths."""
    if not math.isfinite(minimum_turning_radius) or minimum_turning_radius <= 0:
        return "invalid_minimum_turning_radius"
    # Leave space for a car to reverse/turn around outside a coarse graph
    # chord. The graph guides selection; the occupancy footprint proves safety.
    if corridor is None:
        corridor = 2. * minimum_turning_radius + SBR_FRONT_EXTENT
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
    # Admission and FollowPath must validate the same directional sections.
    # A heading window crossing a reversing cusp can hide a sharp terminal
    # turn by adding the next section's distance and cancelling its heading.
    from .motion_path import motion_sections
    for first, last in motion_sections(points):
        reason = _section_curvature_issue(
            points[first:last+1], headings[first:last+1], grid.info.resolution,
            minimum_turning_radius, heading_quantization_bins)
        if reason:
            return reason
    return None


def _section_curvature_issue(points, headings, resolution, minimum_turning_radius,
                             heading_quantization_bins):
    """Check one shared-cusp section using its own heading/arc endpoints."""
    traveled = [0.0]
    for a, b, h0, h1 in zip(points, points[1:], headings, headings[1:]):
        length = distance(a, b)
        turn = abs(math.atan2(math.sin(h1-h0), math.cos(h1-h0)))
        arc = length * turn / (2 * math.sin(turn/2)) if turn > 1e-6 else length
        traveled.append(traveled[-1] + arc)
    # Reeds-Shepp lattice poses can repeat a quantized heading across two
    # half-primitives. A window ending halfway through a primitive must not
    # count its full heading-bin change as continuous curvature. Bound that
    # uncertainty and independently check the actual sampled geometry below.
    bin_angle = 2 * math.pi / heading_quantization_bins if heading_quantization_bins > 0 else 0.
    # A physical connector can have exact continuous headings followed by a
    # Smac suffix. Quantization uncertainty belongs to each endpoint, not the
    # entire path; one continuous heading must not reinterpret all lattice
    # poses as exact, nor grant a full bin to every continuous interval.
    quantized_headings = [bool(bin_angle) and
                          abs(h / bin_angle - round(h / bin_angle)) < 1e-4
                          for h in headings]
    quantized = any(quantized_headings)
    micro_step = min(0.05, max(0.01, resolution * 0.1))
    checked_headings = list(headings)
    if quantized:
        for n in range(1,len(points)):
            turn = math.atan2(math.sin(headings[n]-headings[n-1]),
                              math.cos(headings[n]-headings[n-1]))
            edge=(points[n][0]-points[n-1][0],points[n][1]-points[n-1][1])
            boundary = n in (1,len(points)-1) or (
                n+1 < len(points) and edge[0]*(points[n+1][0]-points[n][0])
                + edge[1]*(points[n+1][1]-points[n][1]) < 0)
            if (boundary and quantized_headings[n-1] and quantized_headings[n]
                    and distance(points[n-1],points[n]) < micro_step
                    and abs(turn) <= bin_angle+1e-6):
                # A sub-cell analytic join can land across one orientation
                # bin without travelling a full motion primitive. Do not
                # interpret that lattice step as a physical centimetre turn.
                # Larger heading jumps are still checked normally.
                checked_headings[n] = checked_headings[n-1]
    # Check heading changes over travelled distance, ignoring zero-length poses.
    for i in range(len(points) - 1):
        j = i + 1
        while j < len(points) - 1 and traveled[j] - traveled[i] < 2.0:
            j += 1
        arc = traveled[j] - traveled[i]
        if arc < 0.5:
            continue
        turn = math.atan2(math.sin(checked_headings[j] - checked_headings[i]),
                          math.cos(checked_headings[j] - checked_headings[i]))
        uncertainty = (bin_angle / 2 *
                       (quantized_headings[i] + quantized_headings[j]))
        bounded_turn = max(0., abs(turn) - uncertainty)
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
            # Smac analytic expansion can append a centimetre-scale connector
            # at a gear cusp. Its quantized heading and tiny lateral mismatch
            # do not define a reliable three-point circle, especially when
            # validating the sliced section again. Skip only fits touching
            # such micro-edges; full heading/footprint checks and the other
            # metre-spaced geometric fits still apply to the entire path.
            if any((n > 0 and distance(points[n-1], points[n]) < micro_step)
                   or (n+1 < len(points) and distance(points[n], points[n+1]) < micro_step)
                   for n in (i,j,k)):
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
