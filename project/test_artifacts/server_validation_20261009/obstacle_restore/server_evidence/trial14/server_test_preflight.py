"""Fail a server trial before recording/motion if its runtime has conflicting owners."""
import argparse
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import time
import yaml


ROLES = {
    'planner': ('planner_server',),
    'controller': ('controller_server',),
    'lifecycle': ('lifecycle_manager',),
    'mapper': ('mapper_node',),
    'graphnav_planner': ('planner_node',),
    'graph_bridge': ('graph_path_bridge',),
    'kinematic': ('kinematic_vehicle_control', 'wildos_nav2_adapter.kinematic_vehicle_control'),
    'official_adapter': ('beamng_wildos_adapter',),
    'goal_standoff': ('goal_standoff',),
    'triangulation': ('obj_mask_triangulation.py',),
    'vision': ('visual_navigation.wildos.nav',),
    'cloud_filter': ('cloud_filter',),
    'pose_graph_publisher': ('dummy_pose_graph_publisher.py',),
    'visualization': ('visualization_bridge',),
    'rviz': ('rviz2',),
}
NODES = ('/planner_server', '/controller_server', '/spot1/mapper_node',
         '/spot1/graphnav_planner', '/wildos_graph_path_bridge',
         '/wildos_kinematic_vehicle_control', '/beamng_wildos_adapter',
         '/beamng_vehicle_goal_standoff', '/obj_mask_triangulator', '/wildos',
         '/rviz', '/beamng_bridge', '/vehicles/ego')


def process_rows():
    rows = []
    for p in Path('/proc').iterdir():
        if not p.name.isdigit() or int(p.name) == os.getpid():
            continue
        try:
            tokens = (p / 'cmdline').read_bytes().decode(errors='replace').split('\0')
            tokens = [t for t in tokens if t]
            if not tokens:
                continue
            env = dict(s.split('=', 1) for s in (p / 'environ').read_bytes()
                       .decode(errors='replace').split('\0') if '=' in s)
            stat = (p / 'stat').read_text().split(') ', 1)[1].split()
            rows.append({'pid': int(p.name), 'ppid': int(stat[1]), 'state': stat[0],
                         'tokens': tokens, 'command': ' '.join(tokens),
                         'domain': env.get('ROS_DOMAIN_ID', '0'),
                         'display': env.get('DISPLAY'), 'cwd': str((p / 'cwd').resolve())})
        except (OSError, ValueError):
            continue
    return rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--session-dir', type=Path, required=True)
    parser.add_argument('--arm', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    errors = []
    report = {'checked_at_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
              'domain': os.environ.get('ROS_DOMAIN_ID', '0'), 'roles': {},
              'leftovers': [], 'errors': errors}
    rows = process_rows()
    domain = report['domain']
    for role, markers in ROLES.items():
        matches = [r for r in rows if r['domain'] == domain
                   and Path(r['tokens'][0]).name not in ('bash', 'sh', 'timeout', 'ssh')
                   and any(t in markers or Path(t).name in markers for t in r['tokens'])]
        report['roles'][role] = matches
        if len(matches) != 1:
            errors.append(f'{role}: expected one process, found {len(matches)}')
        pidfile = args.session_dir / f'{role}.pid'
        if role != 'rviz' and pidfile.exists():
            registered = int(pidfile.read_text().strip())
            if len(matches) == 1 and matches[0]['pid'] != registered:
                errors.append(f'{role}: PID registry does not own the running process')
    for r in rows:
        command, tokens = r['command'], r['tokens']
        old_trial = any(Path(t).name in ('full_trial.py', 'run_trial.py',
                                        'continue_town_memory_trial.py',
                                        'sensor_pose_probe.py',
                                        'stationary_sensor_calibration.py') for t in tokens)
        recording = (('bag' in tokens and 'record' in tokens and r['domain'] == domain)
                     or ('ffmpeg' in Path(tokens[0]).name and str(args.session_dir) in command))
        orphan_probe = (r['ppid'] == 1 and ('ros2 param get ' in command
                        or 'ros2 service call ' in command or 'ros2 topic echo ' in command))
        # A caller's runner is allowed; earlier trials and detached probe shells are not.
        if old_trial and r['pid'] != os.getppid() or recording or orphan_probe:
            report['leftovers'].append(r)
    if report['leftovers']:
        errors.append('Previous test/recorder or orphan diagnostic processes remain')
    usage = shutil.disk_usage(args.session_dir)
    report['disk_free_bytes'] = usage.free
    report['uptime_load'] = subprocess.run(['uptime'], text=True, capture_output=True).stdout.strip()
    report['beamng_listener'] = subprocess.run(
        ['ss', '-ltnp', 'sport = :25252'], text=True, capture_output=True).stdout.strip()
    if 'LISTEN' not in report['beamng_listener']:
        errors.append('BeamNG port 25252 is not listening')
    expected_config = args.arm / 'nav2_adapter/config/nav2_official_control.yaml'
    config_hash = hashlib.sha256(expected_config.read_bytes()).hexdigest()
    report['nav2_config_sha256'] = config_hash
    config = yaml.safe_load(expected_config.read_text())
    for role in ('planner', 'controller'):
        for r in report['roles'][role]:
            tokens = r['tokens']
            if '--params-file' in tokens:
                p = Path(tokens[tokens.index('--params-file') + 1])
                r['config_path'] = str(p)
                r['config_sha256'] = hashlib.sha256(p.read_bytes()).hexdigest()
                if r['config_sha256'] != config_hash:
                    errors.append(f'{role}: running Nav2 config differs from the test arm')
    import rclpy
    from rcl_interfaces.srv import GetParameters
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image, PointCloud2
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node('wildos_server_preflight')
    try:
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        names = [('/' if ns == '/' else ns + '/') + name
                 for name, ns in node.get_node_names_and_namespaces()]
        counts = Counter(names)
        report['ros_nodes'] = dict(counts)
        for name in NODES:
            if counts[name] != 1:
                errors.append(f'{name}: expected one ROS node, found {counts[name]}')
        for name, count in counts.items():
            if count > 1:
                errors.append(f'Duplicate ROS node: {name} ({count})')
        report['topic_publishers'] = {}
        for topic in ('/spot1/odom', '/spot1/realsense/front/color/image_raw', '/wildos/kinematic_state'):
            publishers = node.get_publishers_info_by_topic(topic)
            report['topic_publishers'][topic] = [p.node_namespace + '/' + p.node_name for p in publishers]
            if len(publishers) != 1:
                errors.append(f'{topic}: expected one producer, found {len(publishers)}')
        command_owners = [p.node_name for p in node.get_publishers_info_by_topic('/wildos/cmd_vel')]
        report['command_publishers'] = command_owners
        unexpected = set(command_owners) - {'controller_server', 'wildos_graph_path_bridge'}
        if unexpected:
            errors.append('Unexpected command publisher: ' + ', '.join(sorted(unexpected)))
        # File hashes do not establish what long-running nodes actually loaded.
        # Dynamics bypass must not silently remove planning obstacle constraints.
        report['environment_constraints'] = {}
        expected_runtime = {
            '/controller_server': {
                'FollowPath.use_collision_detection': True},
        }
        for name in ('local_costmap', 'global_costmap'):
            costmap_config = config[name][name]['ros__parameters']
            layer = costmap_config['obstacle_layer']
            radius = costmap_config['robot_radius']
            if not (radius >= costmap_config['resolution']/2.0 and
                    costmap_config['inflation_layer']['inflation_radius'] >= radius):
                errors.append(name + ': invalid grid proxy for RPP collision sampling')
            if set(layer.get('observation_sources', '').split()) != {'cloud', 'clearing_cloud'}:
                errors.append(name + ': configuration has no marking/clearing contract')
            fields = {'obstacle_layer.observation_sources': 'cloud clearing_cloud',
                      'obstacle_layer.enabled': True,
                      'robot_radius': radius,
                      'inflation_layer.inflation_radius': costmap_config['inflation_layer']['inflation_radius']}
            for source, marking, clearing in (('cloud', True, False),
                                              ('clearing_cloud', False, True)):
                options = layer.get(source, {})
                for key, expected in (('topic', '/wildos/points' if marking else '/wildos/clearing_points'),
                                      ('marking', marking), ('clearing', clearing),
                                      ('min_obstacle_height', -1000.0),
                                      ('max_obstacle_height', 1000.0)):
                    if options.get(key) != expected:
                        errors.append(name + ': invalid ' + source + '.' + key)
                    fields['obstacle_layer.' + source + '.' + key] = expected
            expected_runtime['/' + name + '/' + name] = fields
        for owner, fields in expected_runtime.items():
            client = node.create_client(GetParameters, owner + '/get_parameters')
            try:
                if not client.wait_for_service(timeout_sec=3):
                    errors.append(owner + ': environment parameter service unavailable')
                    continue
                future = client.call_async(GetParameters.Request(names=list(fields)))
                rclpy.spin_until_future_complete(node, future, timeout_sec=4)
                if not future.done() or future.exception() or len(future.result().values) != len(fields):
                    errors.append(owner + ': environment parameter query failed')
                    continue
                actual = {}
                for (key, expected), value in zip(fields.items(), future.result().values):
                    if isinstance(expected, bool):
                        observed = value.bool_value if value.type == 1 else None
                    elif isinstance(expected, str):
                        observed = value.string_value if value.type == 4 else None
                    else:
                        observed = value.double_value if value.type == 3 else None
                    actual[key] = observed
                    if observed != expected:
                        errors.append(owner + ': loaded ' + key + ' differs from required contract')
                report['environment_constraints'][owner] = actual
            finally:
                node.destroy_client(client)
        report['controls_before_test'] = {}
        report['sensor_geometry'] = {}
        for owner in ('/beamng_wildos_adapter', '/beamng_vehicle_goal_standoff',
                      '/wildos_graph_path_bridge'):
            client = node.create_client(GetParameters, owner + '/get_parameters')
            try:
                if not client.wait_for_service(timeout_sec=3):
                    errors.append(owner + ': sensor geometry service unavailable')
                    continue
                fields = ['camera_offset']
                if owner != '/beamng_wildos_adapter':
                    fields.append('minimum_camera_target_range')
                future = client.call_async(GetParameters.Request(names=fields))
                rclpy.spin_until_future_complete(node, future, timeout_sec=4)
                if not future.done() or future.exception() or len(future.result().values) != len(fields):
                    errors.append(owner + ': sensor geometry query failed')
                    continue
                values = future.result().values
                offset = list(values[0].double_array_value)
                if values[0].type != 8 or len(offset) != 3 or not all(map(math.isfinite, offset)):
                    errors.append(owner + ': invalid or missing camera calibration')
                    continue
                report['sensor_geometry'][owner] = {'camera_offset': offset}
                if len(fields) > 1:
                    clearance = values[1].double_value
                    if values[1].type != 3 or not math.isfinite(clearance) or clearance <= 0:
                        errors.append(owner + ': invalid observation range')
                    report['sensor_geometry'][owner]['minimum_camera_target_range'] = clearance
            finally:
                node.destroy_client(client)
        geometries = report['sensor_geometry']
        if len(geometries) == 3:
            offsets = [g['camera_offset'] for g in geometries.values()]
            if any(any(abs(a-b) > 1e-6 for a, b in zip(offsets[0], offset)) for offset in offsets[1:]):
                errors.append('Camera TF and observation planners use different calibration')
            if (abs(geometries['/beamng_vehicle_goal_standoff']['minimum_camera_target_range']
                    - geometries['/wildos_graph_path_bridge']['minimum_camera_target_range']) > 1e-6):
                errors.append('Observation planners use different minimum camera ranges')
        for owner, parameter in (('/wildos_kinematic_vehicle_control', 'enable'),
                                 ('/wildos_graph_path_bridge', 'follow_path')):
            client = node.create_client(GetParameters, owner + '/get_parameters')
            if not client.wait_for_service(timeout_sec=3):
                errors.append(owner + ': parameter service unavailable')
                continue
            future = client.call_async(GetParameters.Request(names=[parameter]))
            rclpy.spin_until_future_complete(node, future, timeout_sec=4)
            if not future.done() or future.exception() or not future.result().values:
                errors.append(owner + ': parameter query failed')
                continue
            value = future.result().values[0]
            report['controls_before_test'][owner + '/' + parameter] = value.bool_value
            if value.type != 1 or value.bool_value:
                errors.append(owner + ': motion must be disabled before a fresh test')
            node.destroy_client(client)
        # Discovery and a listening socket do not establish that the simulator
        # is alive. Sample the inputs and the actual actuator health before a run.
        counts = Counter()
        latest_native = {}
        def received(label, message):
            counts[label] += 1
            if label == 'native':
                try:
                    value = json.loads(message.data)
                    if isinstance(value, dict):
                        latest_native.clear()
                        latest_native.update(value)
                except (ValueError, TypeError):
                    pass
        subscriptions = []
        for topic, label, message_type in (
                ('/spot1/odom', 'odom', Odometry),
                ('/spot1/realsense/front/color/image_raw', 'camera', Image),
                ('/wildos/points', 'obstacle_cloud', PointCloud2),
                ('/wildos/clearing_points', 'clearing_cloud', PointCloud2),
                ('/wildos/native_control_status', 'native', String)):
            subscriptions.append(node.create_subscription(message_type, topic,
                lambda message, label=label: received(label, message), qos_profile_sensor_data))
        deadline = time.monotonic() + 3.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        fields = ('wall_time', 'telemetry_ok', 'state_ok', 'actuator_ok',
                  'control_error', 'telemetry_error', 'state_age_sec', 'telemetry_age_sec')
        report['input_health'] = {'received_in_3s': dict(counts),
            'native': {field: latest_native.get(field) for field in fields}}
        for label in ('odom', 'camera', 'native', 'obstacle_cloud', 'clearing_cloud'):
            if not counts[label]:
                errors.append(label + ': no live input during the preflight sample')
        if any(latest_native.get(field) is not True
               for field in ('telemetry_ok', 'state_ok', 'actuator_ok')):
            errors.append('BeamNG telemetry/state/actuator is not healthy')
        if latest_native.get('control_error'):
            errors.append('BeamNG reports a control error')
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    report['passed'] = not errors
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({'passed': report['passed'], 'errors': errors,
                      'report': str(args.output), 'disk_free_gib': round(usage.free / 2**30, 1)}))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
