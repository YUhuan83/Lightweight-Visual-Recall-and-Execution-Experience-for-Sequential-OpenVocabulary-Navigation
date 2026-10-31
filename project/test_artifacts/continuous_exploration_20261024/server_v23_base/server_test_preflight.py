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


DDS_ENVIRONMENT_FIELDS=('RMW_IMPLEMENTATION','ROS_LOCALHOST_ONLY','FASTRTPS_DEFAULT_PROFILES_FILE')


def goal_reset_service_errors(services):
    """Both reset semantics are required; never fall back to a destructive clear."""
    actual=dict(services)
    return [name+': required navigation reset service is missing or has the wrong type'
            for name in ('/spot1/graphnav_planner/clear_goal',
                         '/spot1/graphnav_planner/release_goal')
            if actual.get(name)!=['std_srvs/srv/Trigger']]


def source_endpoint_errors(publishers, subscribers):
    """One physical source/actuator; ROS recorders may observe the wire."""
    errors=[]
    if (len(publishers)!=1 or publishers[0].node_name!='wildos_cmd_vel_to_control'
            or publishers[0].node_namespace!='/' or publishers[0].topic_type!='std_msgs/msg/String'):
        errors.append('Native actuator must have exactly one speed-loop command owner')
    actuators=[p for p in subscribers if p.node_name=='wildos_native_vehicle_control']
    if (len(actuators)!=1 or actuators[0].node_namespace!='/'
            or actuators[0].topic_type!='std_msgs/msg/String'):
        errors.append('Native actuator source protocol subscriber is missing or ambiguous')
    return errors


def controller_endpoint_errors(publishers, subscribers):
    """The physical motion demand must originate at the stamped RPP source."""
    errors=[]
    if (len(publishers)!=1 or publishers[0].node_name!='controller_server'
            or publishers[0].node_namespace!='/' or publishers[0].topic_type!='geometry_msgs/msg/TwistStamped'):
        errors.append('Physical controller requires one stamped controller_server producer')
    speed=[p for p in subscribers if p.node_name=='wildos_cmd_vel_to_control']
    if (len(speed)!=1 or speed[0].node_namespace!='/'
            or speed[0].topic_type!='geometry_msgs/msg/TwistStamped'):
        errors.append('Stamped controller speed-loop subscriber is missing or ambiguous')
    return errors


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
                         'dds_environment':{key:env.get(key) for key in DDS_ENVIRONMENT_FIELDS},
                         'display': env.get('DISPLAY'), 'cwd': str((p / 'cwd').resolve())})
        except (OSError, ValueError):
            continue
    return rows


def required_process_roles(carrier_model, require_visualization=False):
    if carrier_model not in ('ideal_point', 'ackermann'):
        raise ValueError('Unknown carrier model')
    roles = dict(ROLES)
    if not require_visualization:
        roles.pop('visualization')
        roles.pop('rviz')
    if carrier_model == 'ackermann':
        roles.pop('kinematic')
        roles['native_control'] = ('native_vehicle_control', 'wildos_nav2_adapter.native_vehicle_control')
        roles['speed_control'] = ('cmd_vel_to_control', 'wildos_nav2_adapter.cmd_vel_to_control')
    return roles


def dds_process_errors(roles, sensor_rows, expected_profile, *, profile_reader=None):
    """All ROS owners, including the independent sensor bridge, share policy."""
    from wildos_nav2_adapter.dds_profile import environment_errors
    errors=[]
    for role,rows in {**roles,'sensor':sensor_rows}.items():
        if len(rows)!=1:
            errors.append(role+': missing or conflicting DDS owner');continue
        errors.extend(role+': '+error for error in environment_errors(
            rows[0].get('dds_environment',{}),expected_profile,profile_reader=profile_reader))
    return errors


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--session-dir', type=Path, required=True)
    parser.add_argument('--arm', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--carrier-model', choices=('ideal_point', 'ackermann'), default='ideal_point')
    parser.add_argument('--require-visualization', action='store_true',
                        help='Also require RViz and an advancing matched display preview')
    parser.add_argument('--require-local-dds',action='store_true',
                        help='Require the frozen loopback UDP profile on every ROS owner')
    args = parser.parse_args()
    errors = []
    report = {'checked_at_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
              'domain': os.environ.get('ROS_DOMAIN_ID', '0'), 'carrier_model': args.carrier_model, 'roles': {},
              'leftovers': [], 'errors': errors}
    rows = process_rows()
    domain = report['domain']
    roles = required_process_roles(args.carrier_model, args.require_visualization)
    for role, markers in roles.items():
        matches = [r for r in rows if r['domain'] == domain
                   and Path(r['tokens'][0]).name not in ('bash', 'sh', 'timeout', 'ssh')
                   and any(t in markers or Path(t).name in markers for t in r['tokens'])]
        report['roles'][role] = matches
        if len(matches) != 1:
            errors.append(f'{role}: expected one process, found {len(matches)}')
        if any(r['state'] in ('T', 't', 'Z', 'X') for r in matches):
            errors.append(f'{role}: process is stopped, traced or dead')
        pidfile = args.session_dir / f'{role}.pid'
        if role != 'rviz' and pidfile.exists():
            registered = int(pidfile.read_text().strip())
            if len(matches) == 1 and matches[0]['pid'] != registered:
                errors.append(f'{role}: PID registry does not own the running process')
    incompatible = (('kinematic_vehicle_control', 'wildos_nav2_adapter.kinematic_vehicle_control')
                    if args.carrier_model == 'ackermann' else
                    ('native_vehicle_control', 'wildos_nav2_adapter.native_vehicle_control',
                     'cmd_vel_to_control', 'wildos_nav2_adapter.cmd_vel_to_control'))
    conflicts = [r for r in rows if r['domain'] == domain
                 and any(t in incompatible or Path(t).name in incompatible for t in r['tokens'])]
    if conflicts:
        report['incompatible_control_owners'] = conflicts
        errors.append('An incompatible physical/kinematic control backend remains')
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
    if args.require_local_dds:
        sensor_pid=int((args.session_dir/'sensor.pid').read_text())
        sensor_rows=[row for row in rows if row['pid']==sensor_pid and
            any(Path(t).name=='attach_current_vehicle_sensors.py' for t in row['tokens'])]
        profile=args.arm/'nav2_adapter/config/fastdds_local_udp.xml'
        report['dds_profile_sha256']=hashlib.sha256(profile.read_bytes()).hexdigest()
        report['sensor_dds_owner']=sensor_rows
        errors.extend(dds_process_errors(report['roles'],sensor_rows,profile.read_bytes()))
    usage = shutil.disk_usage(args.session_dir)
    report['disk_free_bytes'] = usage.free
    if usage.free < 2**30:
        errors.append('Less than 1 GiB free for bounded interface evidence and arrival images')
    report['uptime_load'] = subprocess.run(['uptime'], text=True, capture_output=True).stdout.strip()
    report['beamng_listener'] = subprocess.run(
        ['ss', '-ltnp', 'sport = :25252'], text=True, capture_output=True).stdout.strip()
    if 'LISTEN' not in report['beamng_listener']:
        errors.append('BeamNG port 25252 is not listening')
    config_name = ('nav2_ackermann_control.yaml' if args.carrier_model == 'ackermann'
                   else 'nav2_official_control.yaml')
    expected_config = args.arm / 'nav2_adapter/config' / config_name
    config_hash = hashlib.sha256(expected_config.read_bytes()).hexdigest()
    report['nav2_config_sha256'] = config_hash
    config = yaml.safe_load(expected_config.read_text())
    if args.carrier_model == 'ackermann':
        # A renamed fixed-node carrier is not real physics. Query the engine
        # now rather than trusting an earlier file or a backend's mode label.
        from wildos_nav2_adapter.physical_vehicle import read_physics_snapshot, physical_snapshot_errors
        bng = None
        try:
            from beamngpy import BeamNGpy
            bng = BeamNGpy('127.0.0.1', 25252, socket_timeout=3).open(launch=False)
            snapshot = read_physics_snapshot(bng)
            report['physics_snapshot'] = snapshot
            errors.extend(physical_snapshot_errors(snapshot))
            if snapshot.get('speed', 99.) > .15:
                errors.append('Physical vehicle is moving before a fresh test')
        except Exception as exc:
            errors.append('Live physical vehicle inspection failed: ' + str(exc)[:240])
        finally:
            if bng is not None:
                bng.disconnect()
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
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from tf2_msgs.msg import TFMessage
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image, PointCloud2
    from std_msgs.msg import String
    from beamng_msgs.msg import StateSensor
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
        report['goal_reset_services'] = node.get_service_names_and_types()
        errors.extend(goal_reset_service_errors(report['goal_reset_services']))
        nodes = list(NODES)
        if not args.require_visualization:
            nodes.remove('/rviz')
        if args.carrier_model == 'ackermann':
            nodes.remove('/wildos_kinematic_vehicle_control')
            nodes.remove('/spot1/mapper_node')
            nodes.append('/spot1/graphnav_mapper')
            nodes.extend(('/wildos_native_vehicle_control', '/wildos_cmd_vel_to_control'))
        for name in nodes:
            if counts[name] != 1:
                errors.append(f'{name}: expected one ROS node, found {counts[name]}')
        for name, count in counts.items():
            if not args.require_visualization and name in ('/rviz', '/wildos_visualization_bridge'):
                continue
            if count > 1:
                errors.append(f'Duplicate ROS node: {name} ({count})')
        report['topic_publishers'] = {}
        state_topic = ('/vehicles/ego/sensors/state' if args.carrier_model == 'ackermann'
                       else '/wildos/kinematic_state')
        publisher_topics = ['/spot1/odom', '/spot1/realsense/front/color/image_raw', state_topic,
                            '/wildos/native_control_status']
        if args.require_visualization:
            publisher_topics.append('/wildos/viz/rgb_target')
        for topic in publisher_topics:
            publishers = node.get_publishers_info_by_topic(topic)
            report['topic_publishers'][topic] = [p.node_namespace + '/' + p.node_name for p in publishers]
            if len(publishers) != 1:
                errors.append(f'{topic}: expected one producer, found {len(publishers)}')
        command_owners = [p.node_name for p in node.get_publishers_info_by_topic('/wildos/cmd_vel')]
        report['command_publishers'] = command_owners
        unexpected = set(command_owners) - {'controller_server', 'wildos_graph_path_bridge'}
        if unexpected:
            errors.append('Unexpected command publisher: ' + ', '.join(sorted(unexpected)))
        if args.carrier_model == 'ackermann':
            from wildos_nav2_adapter.source_control import COMMAND_TOPIC, CONTROLLER_TOPIC, PROTOCOL, receipt_issue
            controller_owners=node.get_publishers_info_by_topic(CONTROLLER_TOPIC)
            controller_readers=node.get_subscriptions_info_by_topic(CONTROLLER_TOPIC)
            report['controller_source_publishers']=[p.node_name for p in controller_owners]
            report['controller_source_subscribers']=[p.node_name for p in controller_readers]
            errors.extend(controller_endpoint_errors(controller_owners,controller_readers))
            owners = node.get_publishers_info_by_topic(COMMAND_TOPIC)
            report['actuator_command_publishers'] = [p.node_name for p in owners]
            receivers = node.get_subscriptions_info_by_topic(COMMAND_TOPIC)
            report['actuator_command_subscribers'] = [p.node_name for p in receivers]
            errors.extend(source_endpoint_errors(owners,receivers))
            if node.get_publishers_info_by_topic('/wildos/native_control'):
                errors.append('Legacy native-control producer is still running')
        # File hashes do not establish what long-running nodes actually loaded.
        # Dynamics bypass must not silently remove planning obstacle constraints.
        report['environment_constraints'] = {}
        from wildos_nav2_adapter.runtime_contract import (navigation_runtime_contract,
            observation_runtime_contract, runtime_parameter_value)
        try:
            expected_runtime = navigation_runtime_contract(config, args.carrier_model)
            observation = yaml.safe_load((args.arm / 'official_adapter/config/beamng_wildos.yaml').read_text())
            expected_runtime.update(observation_runtime_contract(observation))
        except (ValueError, KeyError, TypeError) as exc:
            errors.append('Navigation configuration contract: ' + str(exc))
            expected_runtime = {}
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
                    observed = runtime_parameter_value(value, expected)
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
        control_owner = ('/wildos_cmd_vel_to_control' if args.carrier_model == 'ackermann'
                         else '/wildos_kinematic_vehicle_control')
        for owner, parameter in ((control_owner, 'enable'),
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
        state_health = {'last_stamp_ns': 0, 'advancing_samples': 0,
                        'last_received_monotonic': None, 'finite': False}
        from image_stream_health import ImageStreamHealth
        image_health = ImageStreamHealth()
        def received(label, message):
            counts[label] += 1
            if label in ('camera', 'preview'):
                image_health.observe(label, message.header.stamp.sec*10**9+message.header.stamp.nanosec,
                                     message.header.frame_id, node.get_clock().now().nanoseconds)
            if label == 'state':
                stamp = message.header.stamp.sec*10**9 + message.header.stamp.nanosec
                if stamp > state_health['last_stamp_ns']:
                    state_health['advancing_samples'] += 1
                    state_health['last_received_monotonic'] = time.monotonic()
                    state_health['last_stamp_ns'] = stamp
                state_health['finite'] = all(map(math.isfinite, (
                    message.position.x, message.position.y, message.position.z,
                    message.velocity.x, message.velocity.y, message.velocity.z,
                    message.dir.x, message.dir.y, message.dir.z,
                    message.up.x, message.up.y, message.up.z)))
            if label == 'native':
                try:
                    value = json.loads(message.data)
                    if isinstance(value, dict):
                        latest_native.clear()
                        latest_native.update(value)
                except (ValueError, TypeError):
                    pass
        subscriptions = []
        lidar_origins = []
        if args.carrier_model == 'ackermann':
            def static_received(message):
                lidar_origins.extend(t for t in message.transforms
                                     if t.child_frame_id == 'spot1/lidar_ray_origin')
            subscriptions.append(node.create_subscription(TFMessage, '/tf_static', static_received,
                QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        for topic, label, message_type in (
                ('/spot1/odom', 'odom', Odometry),
                ('/spot1/realsense/front/color/image_raw', 'camera', Image),
                ('/wildos/points', 'obstacle_cloud', PointCloud2),
                ('/wildos/clearing_points', 'clearing_cloud', PointCloud2),
                (state_topic, 'state', StateSensor),
                ('/wildos/native_control_status', 'native', String)):
            subscriptions.append(node.create_subscription(message_type, topic,
                lambda message, label=label: received(label, message), qos_profile_sensor_data))
        if args.require_visualization:
            subscriptions.append(node.create_subscription(Image, '/wildos/viz/rgb_target',
                lambda message: received('preview', message), qos_profile_sensor_data))
        deadline = time.monotonic() + 3.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if args.carrier_model == 'ackermann':
            from wildos_nav2_adapter.sensor_geometry import SBR_LIDAR_OFFSET
            observed_origins = []
            for transform in lidar_origins:
                v = transform.transform.translation; q = transform.transform.rotation
                observed_origins.append(dict(parent=transform.header.frame_id,
                    translation=[v.x, v.y, v.z], rotation=[q.x, q.y, q.z, q.w]))
            report['lidar_ray_origin'] = observed_origins
            if not observed_origins or any(
                    t['parent'] != 'spot1/base_link'
                    or any(not math.isfinite(a) or abs(a-b) > 1e-6
                           for a,b in zip(t['translation'], SBR_LIDAR_OFFSET))
                    or t['rotation'] != [0., 0., 0., 1.] for t in observed_origins):
                errors.append('Physical LiDAR ray origin TF is missing or inconsistent')
        fields = ('wall_time', 'telemetry_ok', 'state_ok', 'actuator_ok',
                  'control_error', 'telemetry_error', 'state_age_sec', 'telemetry_age_sec')
        report['input_health'] = {'received_in_3s': dict(counts),
            'native': {field: latest_native.get(field) for field in fields}, 'state': state_health}
        report['input_health']['image_flow'] = image_health.report(node.get_clock().now().nanoseconds)
        report['input_health']['camera'] = image_health.source_report(node.get_clock().now().nanoseconds)
        if not report['input_health']['camera']['passed']:
            errors.append('Raw RGB is missing, stale, frozen or has invalid source identity')
        if args.require_visualization and not report['input_health']['image_flow']['passed']:
            errors.append('Raw RGB -> preview is missing, stale, frozen or has unmatched source identity')
        for label in ('odom', 'camera', 'native', 'state', 'obstacle_cloud', 'clearing_cloud'):
            if not counts[label]:
                errors.append(label + ': no live input during the preflight sample')
        native_fields = (('telemetry_ok', 'actuator_ok') if args.carrier_model == 'ackermann'
                         else ('telemetry_ok', 'state_ok', 'actuator_ok'))
        if any(latest_native.get(field) is not True for field in native_fields):
            errors.append('BeamNG telemetry/state/actuator is not healthy')
        if args.carrier_model == 'ackermann':
            source_issue = receipt_issue(latest_native.get('source_receipt'), time.time_ns())
            report['input_health']['source_control'] = dict(
                protocol=latest_native.get('control_protocol'), issue=source_issue,
                receipt=latest_native.get('source_receipt'))
            if latest_native.get('control_protocol') != PROTOCOL or source_issue:
                errors.append('Source command receipt is missing, expired or unacknowledged')
        native_wall = latest_native.get('wall_time')
        if (not isinstance(native_wall, (int, float)) or not math.isfinite(native_wall)
                or not -.05 <= time.time()-native_wall <= .5):
            errors.append('Native telemetry publication is stale or has an invalid timestamp')
        if (not state_health['finite'] or state_health['advancing_samples'] < 2
                or state_health['last_received_monotonic'] is None
                or time.monotonic()-state_health['last_received_monotonic'] > .5):
            errors.append('StateSensor is non-finite, stale or not advancing')
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
