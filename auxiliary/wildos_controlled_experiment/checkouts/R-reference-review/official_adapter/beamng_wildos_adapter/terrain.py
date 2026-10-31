"""Measured lower-surface heights for GraphNav, in a gravity-aligned frame."""
import numpy as np
from .mapping import occupancy_from_points
from wildos_nav2_adapter.pointcloud import local_ground_plane, filter_robot_footprint
from wildos_nav2_adapter.vehicle_geometry import SBR_FRONT_EXTENT, SBR_REAR_EXTENT, SBR_HALF_WIDTH


def terrain_layers(points, resolution=.5, width=80, height=80,
                   sensor_origin_xy=(0., 0.), ground_plane_radius=8.,
                   ground_clearance=.25, carrier_model="ackermann"):
    carrier_model = str(carrier_model).strip().lower()
    if carrier_model not in ("ideal_point", "ackermann"):
        raise ValueError("carrier_model must be ideal_point or ackermann")
    points = np.asarray(points, dtype=float)
    occupancy = occupancy_from_points(
        points, resolution, width, height, ground_plane_radius, ground_clearance,
        sensor_origin_xy, clear_robot_footprint=True, carrier_model=carrier_model)
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
    return dict(traversability=np.maximum(occupancy, 0)/100.,
                unknown=(occupancy < 0).astype(float),
                elevation=elevation, elevation_valid=valid)


def grid_map_values(layer):
    """ROS grid_map columns/rows start at the upper right corner."""
    return np.asarray(layer)[::-1, ::-1].ravel().astype(float).tolist()
