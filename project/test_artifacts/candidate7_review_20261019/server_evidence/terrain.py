"""Measured lower-surface heights for GraphNav, in a gravity-aligned frame."""
import numpy as np
from .mapping import occupancy_from_points
from wildos_nav2_adapter.pointcloud import local_ground_plane, filter_robot_footprint
from wildos_nav2_adapter.vehicle_geometry import SBR_FRONT_EXTENT, SBR_REAR_EXTENT, SBR_HALF_WIDTH


def witnessed_step_by_cell(points, resolution=.5, subcell=.1, max_slope=.35,
                           width=80, height=80, minimum_returns=2,
                           envelope_height=.5):
    """Observed rise unexplained by the slope bound, in metres per map cell.

    A single height sample per 0.5 m cell cannot distinguish a vertical step
    from a short ramp. Sub-cell minima/maxima preserve the within-cell vertical
    distribution. Compare measured extrema using their actual XY separation,
    including pairs within one sub-cell. A bin centre distance is not the
    return separation and can make even a continuous plane look stepped.
    This is a slope-relative observation, not proof of a vertical wall or its
    true height. Sparse cells provide no positive evidence.
    """
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError('points must be finite Nx3')
    if (not np.isfinite([resolution, subcell, max_slope, envelope_height]).all()
            or resolution <= 0 or not 0 < subcell <= resolution
            or max_slope < 0 or envelope_height <= 0):
        raise ValueError('terrain dimensions and slope bound are invalid')
    for name, value, minimum in (('width', width, 1), ('height', height, 1),
                                  ('minimum_returns', minimum_returns, 2)):
        if (isinstance(value, (bool, np.bool_)) or not np.isscalar(value)
                or not np.isfinite(value) or int(value) != value or value < minimum):
            raise ValueError(name + ' must be a supported positive integer')
    width, height, minimum_returns = int(width), int(height), int(minimum_returns)
    result = np.zeros((height, width), dtype=float)
    if not len(points):
        return result
    points = points[(points[:, 2] >= -8.) & (points[:, 2] <= 1.2)]
    if not len(points):
        return result
    cells = np.floor(points[:, :2] / resolution + [width / 2, height / 2]).astype(int)
    inside = ((cells[:, 0] >= 0) & (cells[:, 0] < width)
              & (cells[:, 1] >= 0) & (cells[:, 1] < height))
    points, cells = points[inside], cells[inside]
    if not len(points):
        return result
    # Only the vehicle-relevant ground envelope participates. A branch above
    # the low surface must not become a terrain step.
    indices = cells[:, 1] * width + cells[:, 0]
    minima = np.full(width * height, np.inf)
    np.minimum.at(minima, indices, points[:, 2])
    envelope = points[:, 2] <= minima[indices] + envelope_height
    points, indices = points[envelope], indices[envelope]
    if not len(points):
        return result
    # Canonical tie breaking keeps extrema positions independent of packet
    # ordering when multiple returns have exactly the same minimum/maximum Z.
    points = points[np.lexsort((points[:, 2], points[:, 1], points[:, 0]))]
    scaled = points[:, :2] / subcell
    nearest = np.rint(scaled)
    # Decimal bin boundaries such as 5.1/.1 can round one ULP below 51.
    # Correct representation error only; never move a genuinely nearby return.
    tolerance = 4. * np.finfo(float).eps * np.maximum(1., np.abs(scaled))
    scaled = np.where(np.abs(scaled - nearest) <= tolerance, nearest, scaled)
    sub = np.floor(scaled).astype(np.int64)
    unique, inverse = np.unique(sub, axis=0, return_inverse=True)
    counts = np.bincount(inverse, minlength=len(unique))
    lows = np.full(len(unique), np.inf)
    highs = np.full(len(unique), -np.inf)
    np.minimum.at(lows, inverse, points[:, 2])
    np.maximum.at(highs, inverse, points[:, 2])
    low_indices = np.full(len(unique), len(points), dtype=np.int64)
    high_indices = low_indices.copy()
    low_mask = points[:, 2] == lows[inverse]
    high_mask = points[:, 2] == highs[inverse]
    np.minimum.at(low_indices, inverse[low_mask], np.flatnonzero(low_mask))
    np.minimum.at(high_indices, inverse[high_mask], np.flatnonzero(high_mask))
    low_points, high_points = points[low_indices], points[high_indices]
    offset = unique.min(axis=0)
    stride = int(unique[:, 1].max() - offset[1] + 3)
    packed = (unique[:, 0] - offset[0]) * stride + (unique[:, 1] - offset[1])
    steps = np.zeros(len(unique))
    supported = counts >= minimum_returns

    def compare(left, right):
        distance = np.linalg.norm(high_points[left, :2] - low_points[right, :2], axis=1)
        rise = highs[left] - lows[right] - max_slope * distance
        np.maximum.at(steps, left, rise)
        np.maximum.at(steps, right, rise)

    same = np.flatnonzero(supported)
    compare(same, same)
    for dx, dy in ((1, 0), (0, 1)):
        target = unique + (dx, dy)
        target_packed = ((target[:, 0] - offset[0]) * stride
                         + (target[:, 1] - offset[1]))
        positions = np.searchsorted(packed, target_packed)
        possible = positions < len(packed)
        left = np.flatnonzero(possible)
        right = positions[possible]
        exact = packed[right] == target_packed[left]
        left, right = left[exact], right[exact]
        valid = supported[left] & supported[right]
        left, right = left[valid], right[valid]
        if not len(left):
            continue
        compare(left, right)
        compare(right, left)
    if not np.any(steps > 0):
        return result
    columns = np.floor(((unique[:, 0] + .5) * subcell) / resolution
                       + width / 2).astype(int)
    rows = np.floor(((unique[:, 1] + .5) * subcell) / resolution
                    + height / 2).astype(int)
    inside = ((columns >= 0) & (columns < width)
              & (rows >= 0) & (rows < height))
    positive = steps > 0
    keep = inside & positive
    if np.any(keep):
        np.maximum.at(result, (rows[keep], columns[keep]), steps[keep])
    return result


def terrain_layers(points, resolution=.5, width=80, height=80,
                   sensor_origin_xy=(0., 0.), ground_plane_radius=8.,
                   ground_clearance=.25, carrier_model="ackermann",
                   obstacle_min_height=.10, max_slope=.35,
                   envelope_height=.5):
    carrier_model = str(carrier_model).strip().lower()
    if carrier_model not in ("ideal_point", "ackermann"):
        raise ValueError("carrier_model must be ideal_point or ackermann")
    points = np.asarray(points, dtype=float)
    occupancy = occupancy_from_points(
        points, resolution, width, height, ground_plane_radius, ground_clearance,
        sensor_origin_xy, clear_robot_footprint=True, carrier_model=carrier_model,
        obstacle_min_height=obstacle_min_height)
    if carrier_model == "ackermann":
        points = filter_robot_footprint(points, SBR_FRONT_EXTENT, SBR_REAR_EXTENT,
                                        SBR_HALF_WIDTH)
    rows, columns = np.indices(occupancy.shape)
    x = (columns+.5-width/2)*resolution
    y = (rows+.5-height/2)*resolution
    # A fitted plane supplies geometry for unknown-height frontier pixels only.
    # It never makes those pixels measured terrain or increases their confidence.
    plane = local_ground_plane(points, ground_plane_radius)
    elevation = np.zeros_like(x) if plane is None else x*plane[0]+y*plane[1]+plane[2]
    valid = np.zeros_like(x)
    elevation_min = np.zeros_like(x)
    elevation_max = np.zeros_like(x)
    height_span = np.zeros_like(x)
    witnessed_step = witnessed_step_by_cell(
        points, resolution, width=width, height=height, max_slope=max_slope,
        envelope_height=envelope_height)
    points = points[(points[:, 2] >= -8.) & (points[:, 2] <= 1.2)]
    cells = np.floor(points[:, :2]/resolution+[width/2, height/2]).astype(int)
    inside = ((cells[:, 0] >= 0) & (cells[:, 0] < width)
              & (cells[:, 1] >= 0) & (cells[:, 1] < height))
    points, cells = points[inside], cells[inside]
    indices = cells[:, 1]*width+cells[:, 0]
    minima = np.full(width*height, np.inf)
    np.minimum.at(minima, indices, points[:, 2])
    lower = points[:, 2] <= minima[indices]+.15
    counts = np.bincount(indices[lower], minlength=width*height)
    sums = np.bincount(indices[lower], weights=points[lower, 2], minlength=width*height)
    observed = (counts >= 2) & (occupancy.ravel() == 0)
    elevation.ravel()[observed] = sums[observed]/counts[observed]
    valid.ravel()[observed] = 1.
    # Preserve the measured vertical envelope inside each observed cell. The
    # envelope is capped near the low surface so overhanging returns do not
    # become a ground step.
    envelope = points[:, 2] <= minima[indices] + envelope_height
    low = np.full(width*height, np.inf)
    high = np.full(width*height, -np.inf)
    np.minimum.at(low, indices[envelope], points[envelope, 2])
    np.maximum.at(high, indices[envelope], points[envelope, 2])
    elevation_min.ravel()[observed] = low[observed]
    elevation_max.ravel()[observed] = high[observed]
    height_span.ravel()[observed] = high[observed] - low[observed]
    return dict(traversability=np.maximum(occupancy, 0)/100.,
                unknown=(occupancy < 0).astype(float),
                elevation=elevation, elevation_valid=valid,
                elevation_min=elevation_min, elevation_max=elevation_max,
                height_span=height_span, witnessed_step=witnessed_step)


def grid_map_values(layer):
    """ROS grid_map columns/rows start at the upper right corner."""
    return np.asarray(layer)[::-1, ::-1].ravel().astype(float).tolist()
