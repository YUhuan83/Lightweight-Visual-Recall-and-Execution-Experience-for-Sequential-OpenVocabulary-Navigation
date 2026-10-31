"""Transport validation and route costs for remembered observation candidates."""
import json
import math
from .graph_route import cell_cost, distance


def decode_candidates(data, limit=8):
    payload = json.loads(data)
    query = ' '.join(str(payload['query']).casefold().split())
    if not query or not isinstance(payload['views'], list):
        raise ValueError('invalid memory candidates')
    views = []
    ids = set()
    for view in payload['views'][:limit]:
        position = view['position']
        heading = float(view['heading'])
        view_id = str(view['id'])
        if (len(position) != 3 or not all(math.isfinite(float(v)) for v in position)
                or not math.isfinite(heading) or view.get('frame') != 'spot1/odom'
                or not view_id or view_id in ids):
            raise ValueError('invalid memory viewpoint')
        ids.add(view_id)
        views.append(dict(view, id=view_id, position=list(map(float, position)), heading=heading))
    return query, views


def heading_options(start, target, observed_heading, current_heading):
    """Observation heading is preferred; vehicle arrival orientation is flexible."""
    result = []
    for heading in (observed_heading, math.atan2(target[1] - start[1], target[0] - start[0]), current_heading):
        heading = math.atan2(math.sin(heading), math.cos(heading))
        if all(abs(math.atan2(math.sin(heading - other), math.cos(heading - other))) > .1 for other in result):
            result.append(heading)
    return result


def path_cost(points, headings, grid, observed_heading, remaining_distance=0.0):
    """Metres plus obstacle proximity, unknown-space and viewing-turn penalties."""
    cost = max(0.0, remaining_distance)
    for a, b in zip(points, points[1:]):
        length = distance(a, b)
        steps = max(1, math.ceil(length / max(grid.info.resolution, .1)))
        for i in range(1, steps + 1):
            t = i / steps
            value = cell_cost(grid, a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
            penalty = 2.0 if value < 0 else max(0.0, value) / 100.0
            cost += length / steps * (1.0 + penalty)
    if headings:
        cost += .25 * abs(math.atan2(math.sin(headings[-1] - observed_heading),
                                    math.cos(headings[-1] - observed_heading)))
    return cost
