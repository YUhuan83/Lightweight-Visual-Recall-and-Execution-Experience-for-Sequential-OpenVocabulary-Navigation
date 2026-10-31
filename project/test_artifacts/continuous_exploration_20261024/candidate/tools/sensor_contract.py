"""Shared sensor geometry and safety policy; no ROS or third-party imports."""
import ast
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONFIG = json.loads((ROOT / 'config/sensor_contract.json').read_text(encoding='utf-8'))


def road_height(y=0):
    # Evaluate only the generator's pure road_z function, without asset dependencies.
    tree = ast.parse((ROOT / 'tools/build_lane_sample.py').read_text(encoding='utf-8'))
    function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'road_z')
    scope = {}
    exec(compile(ast.Module(body=[function], type_ignores=[]), '<road_z>', 'exec'), scope)
    return scope['road_z'](y)


def intrinsics():
    c = CONFIG
    focal = c['width'] / (2 * math.tan(c['hfov'] / 2))
    return focal, focal, c['width'] / 2, c['height'] / 2


def transforms():
    c = CONFIG
    result = []
    for name, yaw in c['cameras'].items():
        frame = f'camera_{name}_link'
        result.append((c['base_frame'], frame, c['camera_position'],
                       [0, 0, math.sin(yaw / 2), math.cos(yaw / 2)]))
        result.append((frame, f'camera_{name}_optical_frame', [0, 0, 0], c['optical_quaternion']))
    for name in ('lidar', 'imu'):
        result.append((c['base_frame'], f'{name}_link', c[f'{name}_position'], [0, 0, 0, 1]))
    return result


def safe_command(values):
    """Reject the entire command if any axis is nonfinite; ignore unsupported axes."""
    if not all(math.isfinite(v) for v in values):
        return 0.0, 0.0
    return (max(-CONFIG['linear_limit'], min(CONFIG['linear_limit'], values[0])),
            max(-CONFIG['angular_limit'], min(CONFIG['angular_limit'], values[5])))


def watchdog(command, received, now):
    return command if 0 <= now - received < CONFIG['command_timeout'] else (0.0, 0.0)


def bridge_entries():
    entries = []
    streams = [('/clock', '/clock', 'rosgraph_msgs/msg/Clock', 'Clock', 'GZ_TO_ROS'),
               ('/sim/cmd_vel', '/sim/cmd_vel', 'geometry_msgs/msg/Twist', 'Twist', 'ROS_TO_GZ')]
    for topic, ros_type, gz_type in [('imu', 'sensor_msgs/msg/Imu', 'IMU'),
                                      ('lidar/points', 'sensor_msgs/msg/PointCloud2', 'PointCloudPacked'),
                                      ('wheel_odom', 'nav_msgs/msg/Odometry', 'Odometry')]:
        streams.append((f'/sim/{topic}', f'/sim/{topic}', ros_type, gz_type, 'GZ_TO_ROS'))
    for name in CONFIG['cameras']:
        streams.append((f'/sim/camera/{name}/image_raw', f'/sim/camera/{name}',
                        'sensor_msgs/msg/Image', 'Image', 'GZ_TO_ROS'))
    for ros, gz, ros_type, gz_type, direction in streams:
        entries.append(dict(ros_topic_name=ros, gz_topic_name=gz, ros_type_name=ros_type,
                            gz_type_name=f'ignition.msgs.{gz_type}', direction=direction))
    return entries
