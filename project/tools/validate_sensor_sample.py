"""Stdlib offline contract tests; --live adds a read-only ROS graph/data check."""
import argparse
import json
import math
import time
import unittest
import xml.etree.ElementTree as ET

from sensor_contract import CONFIG as C, ROOT, bridge_entries, intrinsics, road_height
from sensor_contract import safe_command, transforms, watchdog


class ContractTests(unittest.TestCase):
    def test_geometry_and_scene(self):
        world = ET.parse(ROOT / 'assets/worlds/lane_sensor_sample.sdf').getroot().find('world')
        original = ET.parse(ROOT / 'assets/worlds/lane_sample.sdf').getroot().find('world')
        for tree in (original, world):
            for include in tree.findall('include'):
                include.tail = None
        self.assertEqual([ET.tostring(n) for n in original.findall('include')],
                         [ET.tostring(n) for n in world.findall('include')])
        self.assertFalse(original.findall('.//sensor'))
        self.assertEqual(len(world.findall('model')), 1)
        rover = world.find("model[@name='survey_rover']")
        for actual, expected in zip(map(float, rover.findtext('pose').split()),
                                    [-18., 0., road_height() + .45, 0., 0., 0.]):
            self.assertAlmostEqual(actual, expected)
        plugins = {p.get('name'): p for p in world.findall('plugin')}
        self.assertEqual(plugins['ignition::gazebo::systems::Sensors'].findtext('render_engine'), 'ogre2')
        self.assertIn('ignition::gazebo::systems::Imu', plugins)
        sensors = rover.findall("link[@name='base_link']/sensor")
        self.assertEqual(len(sensors), 5)
        for sensor in sensors:
            name = sensor.get('name')
            if name.startswith('camera_'):
                side = name[7:]
                expected = [*C['camera_position'], 0, 0, C['cameras'][side]]
                frame, topic = f'camera_{side}_optical_frame', f'/sim/camera/{side}'
                self.assertEqual(sensor.get('type'), 'camera')
                self.assertEqual(float(sensor.findtext('camera/horizontal_fov')), C['hfov'])
                for key in ('width', 'height'):
                    self.assertEqual(int(sensor.findtext(f'camera/image/{key}')), C[key])
            else:
                expected = [*C[f'{name}_position'], 0, 0, 0]
                frame, topic = f'{name}_link', f'/sim/{name}'
                self.assertEqual(sensor.get('type'), {'imu': 'imu', 'lidar': 'gpu_lidar'}[name])
            self.assertEqual(list(map(float, sensor.findtext('pose').split())), expected)
            self.assertEqual(sensor.findtext('ignition_frame_id'), frame)
            self.assertEqual(sensor.findtext('topic'), topic)
        drive = rover.find('plugin')
        for key, value in dict(topic='/sim/cmd_vel', odom_topic='/sim/wheel_odom',
                               tf_topic='/sim/wheel_tf', frame_id=C['odom_frame'],
                               child_frame_id=C['base_frame']).items():
            self.assertEqual(drive.findtext(key), value)

    def test_bridge(self):
        entries = json.loads((ROOT / 'config/lane_bridge.yaml').read_text(encoding='utf-8'))
        self.assertEqual(entries, bridge_entries())
        self.assertEqual(len({e['ros_topic_name'] for e in entries}), len(entries))
        self.assertFalse(any(e['ros_topic_name'].startswith('/rover') or
                             'tf' in e['ros_topic_name'] for e in entries))
        self.assertEqual([e['ros_topic_name'] for e in entries if e['direction'] == 'ROS_TO_GZ'],
                         ['/sim/cmd_vel'])

    def test_optical_and_tf(self):
        def rotate(q, v):
            x, y, z, w = q
            cross = (y*v[2]-z*v[1], z*v[0]-x*v[2], x*v[1]-y*v[0])
            second = (y*cross[2]-z*cross[1], z*cross[0]-x*cross[2], x*cross[1]-y*cross[0])
            return [v[i] + 2*w*cross[i] + 2*second[i] for i in range(3)]
        q = C['optical_quaternion']
        for axis, expected in [([1, 0, 0], [0, -1, 0]),
                               ([0, 1, 0], [0, 0, -1]), ([0, 0, 1], [1, 0, 0])]:
            self.assertEqual(rotate(q, axis), expected)
        edges = transforms()
        parents = {child: parent for parent, child, _, _ in edges}
        self.assertEqual(len(parents), 8)
        for parent, child, _, quat in edges:
            self.assertAlmostEqual(sum(v*v for v in quat), 1)
            seen = set()
            while child in parents:
                self.assertNotIn(child, seen)
                seen.add(child)
                child = parents[child]
            self.assertEqual(child, 'base_link')
        for name, expected in [('front', [1, 0, 0]), ('left', [0, 1, 0]), ('right', [0, -1, 0])]:
            mount = next(e[3] for e in edges if e[1] == f'camera_{name}_link')
            for actual, want in zip(rotate(mount, rotate(q, [0, 0, 1])), expected):
                self.assertAlmostEqual(actual, want)

    def test_intrinsics(self):
        fx, fy, cx, cy = intrinsics()
        self.assertEqual((C['width'], C['height'], C['hfov']), (960, 540, 1.35))
        self.assertEqual((cx, cy), (480, 270))
        self.assertEqual(fx, fy)
        self.assertAlmostEqual(2 * math.atan(C['width'] / (2 * fx)), 1.35)

    def test_safety(self):
        self.assertEqual(safe_command([2, 99, -99, 4, 5, -2]), (1, -1))
        self.assertEqual(safe_command([.3, 9, 9, 9, 9, -.2]), (.3, -.2))
        for bad in (float('nan'), float('inf'), -float('inf')):
            for axis in range(6):
                values = [1.] * 6
                values[axis] = bad
                self.assertEqual(safe_command(values), (0, 0))
        for age in (.5, .6, 100, -1):
            self.assertEqual(watchdog((1, 1), 10, 10 + age), (0, 0))
        self.assertEqual(watchdog((1, 1), 10, 10.49), (1, 1))


def live_check(seconds):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    node = rclpy.create_node('lane_sensor_health_check')
    samples, static, dynamic, handles = {}, {}, set(), []
    expected = {'/rover/imu': (Imu, 'imu_link'), '/rover/lidar/points': (PointCloud2, 'lidar_link'),
                '/rover/wheel_odom': (Odometry, C['odom_frame']), '/clock': (Clock, None)}
    for name in C['cameras']:
        for suffix, cls in [('image_raw', Image), ('camera_info', CameraInfo)]:
            expected[f'/rover/camera/{name}/{suffix}'] = (cls, f'camera_{name}_optical_frame')

    def receive(topic, msg):
        values = samples.setdefault(topic, [])
        values.append(msg)
        del values[:-100]

    for topic, (cls, _) in expected.items():
        handles.append(node.create_subscription(cls, topic, lambda msg, t=topic: receive(t, msg), qos_profile_sensor_data))

    def tf_receive(msg):
        for tf in msg.transforms:
            static[tf.child_frame_id] = tf

    handles.append(node.create_subscription(TFMessage, '/tf_static', tf_receive,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)))
    handles.append(node.create_subscription(TFMessage, '/tf',
        lambda msg: dynamic.update((t.header.frame_id, t.child_frame_id) for t in msg.transforms), qos_profile_sensor_data))
    try:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        errors = []
        for topic, (_, frame) in expected.items():
            values = samples.get(topic, [])
            if len(values) < 2:
                errors.append(f'{topic}: fewer than two samples')
                continue
            stamps = [(m.header.stamp.sec, m.header.stamp.nanosec) if frame else
                      (m.clock.sec, m.clock.nanosec) for m in values]
            if not stamps[-1] > stamps[0]:
                errors.append(f'{topic}: clock/stamps not advancing')
            if frame and any(m.header.frame_id != frame for m in values):
                errors.append(f'{topic}: incorrect frame')
            if frame and not any(p.qos_profile.reliability == ReliabilityPolicy.RELIABLE
                                 for p in node.get_publishers_info_by_topic(topic)):
                errors.append(f'{topic}: no reliable publisher')
        for name in C['cameras']:
            images = samples.get(f'/rover/camera/{name}/image_raw', [])
            infos = samples.get(f'/rover/camera/{name}/camera_info', [])
            image_stamps = {(m.header.stamp.sec, m.header.stamp.nanosec) for m in images}
            if not any((m.header.stamp.sec, m.header.stamp.nanosec) in image_stamps for m in infos):
                errors.append(f'{name}: no matched image/CameraInfo stamps')
            fx, fy, cx, cy = intrinsics()
            if any((m.width, m.height) != (C['width'], C['height']) or
                   any(abs(a-b) > 1e-8 for a, b in zip(m.k, [fx, 0, cx, 0, fy, cy, 0, 0, 1])) for m in infos):
                errors.append(f'{name}: calibration mismatch')
        for parent, child, xyz, quat in transforms():
            tf = static.get(child)
            if tf is None:
                errors.append(f'missing static TF {child}')
                continue
            t, q = tf.transform.translation, tf.transform.rotation
            actual = [t.x, t.y, t.z, q.x, q.y, q.z, q.w]
            if tf.header.frame_id != parent or any(abs(a-b) > 1e-8 for a, b in zip(actual, [*xyz, *quat])):
                errors.append(f'incorrect static TF {child}')
        if node.count_publishers('/rover/odom') or ('map', 'odom') in dynamic or any(
                child == C['base_frame'] for _, child in dynamic):
            errors.append('unexpected localization/dynamic base TF; check for debug mode or external nodes')
        for entry in bridge_entries():
            topic = entry['ros_topic_name']
            endpoints = (node.get_subscriptions_info_by_topic(topic) if entry['direction'] == 'ROS_TO_GZ'
                         else node.get_publishers_info_by_topic(topic))
            if not any(e.topic_type == entry['ros_type_name'] for e in endpoints):
                errors.append(f'missing bridge endpoint {topic}')
        clouds = samples.get('/rover/lidar/points', [])
        if clouds:
            names = [f.name for f in clouds[-1].fields]
            print('PointCloud2 fields:', names, '; absent ring/time:', sorted({'ring', 'time'} - set(names)))
        if errors:
            raise RuntimeError('\n'.join(errors))
        print('Live graph, timestamps, calibration, QoS and static TF checks passed (read-only).')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--live', action='store_true')
    parser.add_argument('--seconds', type=float, default=8)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('--seconds must be positive and finite')
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(ContractTests))
    if not result.wasSuccessful():
        raise SystemExit(1)
    if args.live:
        live_check(args.seconds)
