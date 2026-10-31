"""Inspect and stage a bounded server test; no scenario reloads."""
from pathlib import Path
import json
import os
import subprocess
import sys
import time

ROOT = Path('/home/ly/full_town_memory_20261002')
WS = Path('/home/ly/beamng_ros2_0394_ws')
ROOT.mkdir(parents=True, exist_ok=True)


def runtime_env(pid=2284240, domain=40):
    allowed = {'AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH', 'CMAKE_PREFIX_PATH',
               'LD_LIBRARY_PATH', 'PATH', 'PYTHONPATH', 'ROS_DISTRO', 'ROS_VERSION',
               'ROS_PYTHON_VERSION', 'RMW_IMPLEMENTATION', 'ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY'}
    environment = dict(os.environ)
    source = Path(f'/proc/{pid}/environ')
    if not source.exists() and (ROOT / 'speed_control.pid').exists():
        source = Path('/proc', (ROOT / 'speed_control.pid').read_text().strip(), 'environ')
    if not source.exists():
        environment.update(json.loads((ROOT / 'runtime_env.json').read_text()))
        raw = []
    else:
        raw = source.read_bytes().split(b'\0')
    for value in raw:
        if b'=' in value:
            key, content = value.split(b'=', 1)
            if key.decode() in allowed:
                environment[key.decode()] = content.decode()
    environment['ROS_DOMAIN_ID'] = str(domain)
    environment['ROS_LOCALHOST_ONLY'] = '1'
    environment['ROS2CLI_NO_DAEMON'] = '1'
    (ROOT / 'runtime_env.json').write_text(json.dumps({k: environment[k] for k in allowed if k in environment}, indent=2))
    return environment


def ros_environment():
    environment = runtime_env()
    os.environ.update(environment)
    for item in environment.get('PYTHONPATH', '').split(':'):
        if item and item not in sys.path:
            sys.path.insert(0, item)
    return environment


def inspect():
    prefix = sys.argv[2] if len(sys.argv)>2 and sys.argv[2]!='--runtime-child' else 'initial'
    environment = ros_environment()
    from beamngpy import BeamNGpy
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry, OccupancyGrid
    from std_msgs.msg import String
    from sensor_msgs.msg import Image
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
    from rcl_interfaces.srv import SetParameters
    rclpy.init()
    node = Node('town_session_inspect_20261002')
    result = {}
    received = {}
    node.create_subscription(Odometry, '/spot1/odom', lambda m: received.update(odom=m), qos_profile_sensor_data)
    node.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: received.update(grid=m), 10)
    node.create_subscription(String, '/wildos/native_control_status', lambda m: received.update(native=json.loads(m.data)), 10)
    node.create_subscription(String, '/wildos/plan_status', lambda m: received.update(status=json.loads(m.data)), 10)
    node.create_subscription(Image, '/spot1/realsense/front/color/image_raw', lambda m: received.update(camera=m), 1)
    pub = node.create_publisher(Twist, '/wildos/cmd_vel', 1)
    for remote, name in [('/wildos_graph_path_bridge', 'follow_path'), ('/wildos_cmd_vel_to_control', 'enable')]:
        client = node.create_client(SetParameters, remote + '/set_parameters')
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError('Missing parameter service ' + remote)
        p = Parameter(name=name, value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=False))
        f = client.call_async(SetParameters.Request(parameters=[p]))
        rclpy.spin_until_future_complete(node, f, timeout_sec=3)
        if not f.done() or not all(r.successful for r in f.result().results):
            raise RuntimeError('Cannot stop execution: ' + remote)
    pub.publish(Twist())
    end = time.monotonic() + 5
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.05)
    result['received'] = list(received)
    result['native'] = received.get('native')
    result['status'] = received.get('status')
    if 'odom' in received:
        m = received['odom']
        result['odom'] = dict(frame=m.header.frame_id, position=[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z])
    if 'grid' in received:
        m = received['grid']
        result['grid'] = dict(frame=m.header.frame_id, width=m.info.width, height=m.info.height,
                              resolution=m.info.resolution, occupied=sum(v >= 99 for v in m.data))
    if 'camera' in received:
        m = received['camera']
        import cv2
        import numpy as np
        image = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
        cv2.imwrite(str(ROOT / (prefix + '_camera.png')), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        result['camera'] = dict(width=m.width, height=m.height, encoding=m.encoding)
    result['publishers'] = {topic: node.count_publishers(topic) for topic in ('/control', '/wildos/native_control', '/wildos/cmd_vel')}
    result['nodes'] = node.get_node_names_and_namespaces()
    bng = BeamNGpy('127.0.0.1', 25252, socket_timeout=5)
    bng.open(launch=False)
    vehicle = bng.vehicles.get_current()['ego']
    vehicle.connect(bng)
    vehicle.sensors.poll()
    result['beamng'] = dict(position=vehicle.state['pos'], direction=vehicle.state['dir'], time=vehicle.state['time'])
    bng.control.pause()
    result['paused'] = True
    vehicle.disconnect()
    bng.disconnect()
    (ROOT / (prefix + '_state.json')).write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2), flush=True)
    node.destroy_node()
    rclpy.shutdown()


def capture_blocked():
    ros_environment()
    import rclpy
    import numpy as np
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry, OccupancyGrid
    from sensor_msgs.msg import PointCloud2
    from beamng_wildos_adapter.pointcloud import decode_xyz
    from wildos_nav2_adapter.graph_route import footprint_blockers
    from wildos_nav2_adapter.graph_path_bridge import xy, yaw
    import wildos_nav2_adapter.graph_path_bridge as module
    rclpy.init()
    node = Node('town_blocked_pose_capture')
    received = {}
    for key, kind, topic in [('odom', Odometry, '/spot1/odom'),
                              ('global', OccupancyGrid, '/global_costmap/costmap'),
                              ('local', OccupancyGrid, '/local_costmap/costmap'),
                              ('raw', PointCloud2, '/spot1/ouster/front/points_filtered'),
                              ('marking', PointCloud2, '/wildos/points'),
                              ('clearing', PointCloud2, '/wildos/clearing_points')]:
        node.create_subscription(kind, topic, lambda m, key=key: received.update({key: m}),
                                 qos_profile_sensor_data if key in ('odom', 'raw') else 10)
    end = time.monotonic() + 6
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.05)
    arrays = {}
    summary = {'module_file': module.__file__, 'received': list(received)}
    if 'odom' in received:
        pose = received['odom'].pose.pose
        summary['position'] = list(xy(pose))
        summary['heading'] = yaw(pose)
        for key in ('global', 'local'):
            if key not in received:
                continue
            grid = received[key]
            arrays[key] = np.asarray(grid.data, dtype=np.int16).reshape(grid.info.height, grid.info.width)
            summary[key] = dict(resolution=grid.info.resolution,
                origin=[grid.info.origin.position.x, grid.info.origin.position.y],
                blockers=footprint_blockers(grid, xy(pose), yaw(pose)))
    for key in ('raw', 'marking', 'clearing'):
        if key in received:
            cloud = received[key]
            arrays[key] = decode_xyz(cloud)
            summary[key] = dict(frame=cloud.header.frame_id, points=len(arrays[key]),
                stamp=[cloud.header.stamp.sec, cloud.header.stamp.nanosec])
    np.savez_compressed(ROOT / 'blocked_pose_snapshot.npz', **arrays)
    (ROOT / 'blocked_pose_snapshot.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2), flush=True)
    node.destroy_node()
    rclpy.shutdown()


def rates():
    ros_environment()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry, OccupancyGrid
    from sensor_msgs.msg import Image, PointCloud2
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    rclpy.init()
    node = Node('town_measurement_rates')
    values = {}
    def callback(key, m):
        now = node.get_clock().now().nanoseconds/1e9
        value = values.setdefault(key, {'count':0, 'arrivals':[], 'ages':[]})
        value['count'] += 1
        value['arrivals'].append(now)
        if hasattr(m, 'header'):
            value['ages'].append(now-m.header.stamp.sec-m.header.stamp.nanosec/1e9)
        if isinstance(m, String): value['last'] = m.data[:400]
    for key, kind, topic in [('odom', Odometry, '/spot1/odom'),
            ('grid', OccupancyGrid, '/global_costmap/costmap'), ('local', OccupancyGrid, '/local_costmap/costmap'),
            ('image', Image, '/spot1/realsense/front/color/image_raw'),
            ('cloud', PointCloud2, '/wildos/points'), ('rawgoal', PoseStamped, '/spot1/raw_object_waypoint'),
            ('standoff', PoseStamped, '/spot1/imgnav_waypoint'),
            ('native', String, '/wildos/native_control_status')]:
        node.create_subscription(kind, topic, lambda m, key=key: callback(key, m), qos_profile_sensor_data)
    end = time.monotonic()+8
    while time.monotonic()<end: rclpy.spin_once(node, timeout_sec=.05)
    for value in values.values():
        arrivals = value.pop('arrivals')
        value['max_gap'] = max((b-a for a,b in zip(arrivals,arrivals[1:])), default=None)
        ages = value.pop('ages')
        if ages: value['age_range']=[min(ages),max(ages)]
    print(json.dumps(values, indent=2), flush=True)
    (ROOT/'measurement_rates.json').write_text(json.dumps(values, indent=2))
    node.destroy_node()
    rclpy.shutdown()


def planner_probe():
    """Request real Smac paths at the held failed pose, without executing them."""
    ros_environment()
    import math
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry, OccupancyGrid
    from nav2_msgs.action import ComputePathToPose
    from wildos_nav2_adapter.graph_path_bridge import xy, yaw
    from wildos_nav2_adapter.graph_route import footprint_blockers, validate_vehicle_path, cell_cost
    rclpy.init()
    node = Node('held_pose_readonly_planner_probe')
    received = {}
    node.create_subscription(Odometry, '/spot1/odom', lambda m: received.update(odom=m), qos_profile_sensor_data)
    node.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: received.update(grid=m), 10)
    end = time.monotonic() + 5
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.05)
    pose, grid = received['odom'].pose.pose, received['grid']
    start = xy(pose)
    report = dict(start=list(start), heading=yaw(pose), center_cost=cell_cost(grid, *start),
                  bridge_start_blockers=footprint_blockers(grid, start, yaw(pose)), plans=[])
    client = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
    if not client.wait_for_server(timeout_sec=5):
        raise RuntimeError('Smac planner unavailable')
    for heading in (2.967, yaw(pose), 0.):
        goal = ComputePathToPose.Goal()
        goal.planner_id = 'CarLike'
        goal.use_start = True
        goal.start.header = received['odom'].header
        goal.start.pose = pose
        goal.goal.header = received['odom'].header
        goal.goal.pose.position.x, goal.goal.pose.position.y = 734.294, -11.946
        goal.goal.pose.orientation.z, goal.goal.pose.orientation.w = math.sin(heading/2), math.cos(heading/2)
        sent = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, sent, timeout_sec=5)
        if not sent.done() or not sent.result().accepted:
            raise RuntimeError('Planner request not accepted')
        handle = sent.result()
        finished = handle.get_result_async()
        rclpy.spin_until_future_complete(node, finished, timeout_sec=12)
        if not finished.done():
            handle.cancel_goal_async()
            report['plans'].append(dict(heading=heading, status='timeout'))
            continue
        result = finished.result()
        points = [xy(p.pose) for p in result.result.path.poses]
        headings = [yaw(p.pose) for p in result.result.path.poses]
        reason = validate_vehicle_path(points, headings, [start, (734.294, -11.946)], grid,
                                       (734.294, -11.946), start=start,
                                       target_heading=heading, heading_quantization_bins=72)
        report['plans'].append(dict(heading=heading, status=int(result.status), poses=len(points),
            points=points, headings=headings, bridge_validation=reason))
    name = sys.argv[2] if len(sys.argv)>2 and sys.argv[2] != '--runtime-child' else 'planner_probe'
    (ROOT / (name + '.json')).write_text(json.dumps(report, indent=2))
    print(json.dumps({**report, 'plans': [{k:v for k,v in p.items() if k not in ('points','headings')} for p in report['plans']]}, indent=2), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    if '--runtime-child' not in sys.argv:
        raise SystemExit(subprocess.run([sys.executable, __file__, *sys.argv[1:], '--runtime-child'],
                                       env=runtime_env()).returncode)
    if sys.argv[1] == 'inspect':
        inspect()
    elif sys.argv[1] == 'capture_blocked':
        capture_blocked()
    elif sys.argv[1] == 'planner_probe':
        planner_probe()
    elif sys.argv[1] == 'rates':
        rates()
