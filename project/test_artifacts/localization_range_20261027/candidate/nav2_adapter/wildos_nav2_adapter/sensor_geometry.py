"""Measured rig geometry, expressed from the existing ROS navigation origin."""
import math

# ego_front_cam on the current SBR rig; measured with BeamNG sensor APIs while
# paused (trial12 stationary_sensor_calibration.json). Other rigs must override.
SBR_CAMERA_OFFSET = (2.751995092, 0.000008318, 2.209999084)
# Physical rig front_lidar; 2026-10-15 SDK observations with <0.1 mm
# vehicle translation between bracketing pose reads. This is the ray origin,
# not the virtual StateSensor frame in which the adapter encodes cloud points.
SBR_LIDAR_OFFSET = (1.252, 0.0, 1.560)


def observation_clearance(camera_offset, minimum_range, endpoint_tolerance):
    """Conservative planar sensor clearance, independent of collision footprint."""
    if (len(camera_offset) != 3 or not all(map(math.isfinite, camera_offset))
            or not math.isfinite(minimum_range) or minimum_range <= 0
            or not math.isfinite(endpoint_tolerance) or endpoint_tolerance < 0):
        raise ValueError('invalid observation geometry')
    return math.hypot(camera_offset[0], camera_offset[1]) + minimum_range + endpoint_tolerance


def geometry_restart_reason(parameters, get_parameter):
    """Rigid transforms/observation bounds must change together on restart."""
    for parameter in parameters:
        if parameter.name in ('camera_offset', 'lidar_offset', 'base_offset', 'minimum_camera_target_range'):
            if parameter.value != get_parameter(parameter.name).value:
                return parameter.name + ' requires a coordinated sensor/planner restart'
    return None
