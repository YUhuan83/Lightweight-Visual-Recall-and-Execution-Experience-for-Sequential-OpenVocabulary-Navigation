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
        if 'horizontal_fov' in view and (not math.isfinite(float(view['horizontal_fov']))
                or not 0 < float(view['horizontal_fov']) <= math.pi):
            raise ValueError('invalid remembered field of view')
        ids.add(view_id)
        object_position = view.get('object_position')
        if object_position is not None:
            object_radius = float(view.get('object_radius', 0.0))
            if (len(object_position) != 3 or not all(math.isfinite(float(v)) for v in object_position)
                    or not math.isfinite(object_radius) or object_radius < 0):
                raise ValueError('invalid remembered object region')
        normalized = dict(view, id=view_id, position=list(map(float, position)), heading=heading)
        # Validation and consumers must see the same numeric representation.
        # Accepting float-convertible coordinates but retaining JSON strings
        # previously failed later in distance/observation calculations.
        if object_position is not None:
            normalized['object_position'] = list(map(float, object_position))
            normalized['object_radius'] = object_radius
        if 'horizontal_fov' in view:
            normalized['horizontal_fov'] = float(view['horizontal_fov'])
        views.append(normalized)
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
