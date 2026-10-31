"""Reproduce a recorded positive view; this is not the full Town sequence."""
from pathlib import Path
import os
import json
import sys

ROOT = Path('/home/ly/town_memory_validation_20261003')
os.environ.update(json.loads(Path('/home/ly/full_town_memory_20261002/runtime_env.json').read_text()))
os.environ.update(ROS_DOMAIN_ID='40', ROS_LOCALHOST_ONLY='1', ROS2CLI_NO_DAEMON='1',
                  WILDOS_TEST_SESSION_DIR=str(ROOT))
for item in os.environ.get('PYTHONPATH', '').split(':'):
    if item and item not in sys.path:
        sys.path.insert(0, item)
# Reuse only the reviewed harness definition, never its multi-goal main loop.
source = (ROOT/'traffic_light_station_test.py').read_text()
exec(compile(source.split('\nn = Harness()')[0], str(ROOT/'traffic_light_station_test.py'), 'exec'))
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from rclpy.serialization import serialize_message
from tf2_ros import Buffer, TransformListener
sys.path.insert(0, '/home/ly/beamng_ros2_0394_ws/src/nebula2-wildos/visual_navigation')
from visual_navigation.wildos.target_bearing import mask_bearing

n = Harness()
n.phase = 'bearing_focus'
clouds = deque(maxlen=5)
tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer, n)
bearings = []
saved_pairs = 0
n.create_subscription(PointCloud2, '/spot1/ouster/front/points_filtered',
                      clouds.append, qos_profile_sensor_data)


def record_bearing(message):
    global saved_pairs
    if message.query != n.query:
        return
    dims = [d.size for d in message.object_mask.layout.dim]
    mask = np.asarray(message.object_mask.data[message.object_mask.layout.data_offset:],
                      dtype=np.uint8).reshape(dims)[0, 0]
    t = message.cam_transforms.transforms[0].transform
    rotation = Rotation.from_quat([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]).as_matrix()
    heading = mask_bearing(mask, np.asarray(message.cam_infos[0].k).reshape(3, 3), rotation)
    if heading is None:
        return
    stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
    if stamp < n.query_started_ns:
        return
    entry = dict(bearing=heading, observer=[t.translation.x, t.translation.y],
                 stamp_ns=stamp, pixels=int(mask.sum()))
    bearings.append(entry)
    if len(bearings) % 10 == 1:
        n.event('bearing_evidence', **entry)
    if saved_pairs >= 3 or not clouds:
        return
    cloud = min(clouds, key=lambda c: abs(c.header.stamp.sec*10**9+c.header.stamp.nanosec-stamp))
    if abs(cloud.header.stamp.sec*10**9+cloud.header.stamp.nanosec-stamp) > 200_000_000:
        return
    try:
        transform = tf_buffer.lookup_transform('spot1/odom', cloud.header.frame_id,
                                               rclpy.time.Time.from_msg(cloud.header.stamp))
    except Exception:
        return
    prefix = out/f'pair_{saved_pairs}'
    prefix.with_suffix('.mask.cdr').write_bytes(serialize_message(message))
    prefix.with_suffix('.cloud.cdr').write_bytes(serialize_message(cloud))
    prefix.with_suffix('.tf.cdr').write_bytes(serialize_message(transform))
    saved_pairs += 1


n.create_subscription(ObjectMaskWithTf, '/spot1/object_mask', record_bearing, 10)
report = {'scope': 'focused positive-view approach diagnostic', 'full_sequence_passed': False,
          'map_reload_count': 0, 'setup_teleports': 1, 'lidar_range_m': 40.0}
try:
    subprocess.run([sys.executable, str(ROOT/'restart_component.py'), 'graph_bridge'], check=True, timeout=35)
    n.stop()
    samples = [json.loads(l) for l in (ROOT/'traffic_light_station_19_offline_repair_20261004/native_samples.jsonl').read_text().splitlines()]
    sample = min(samples, key=lambda row: abs(row['wall_time']-1791078725.6652849))
    forward = np.asarray(sample['direction']); forward /= np.linalg.norm(forward)
    right = np.cross(forward, sample['up']); right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    # BeamNG teleport applies the inverse quaternion to the canonical -Y front.
    quaternion = Rotation.from_matrix(np.column_stack((-right, -forward, up))).inv().as_quat()
    position = list(sample['position']); position[2] += .1
    b.control.pause()
    b.vehicles.teleport('ego', position, quaternion.tolist(), reset=True)
    b.control.resume()
    n.spin(2.)
    for component in ('mapper', 'graphnav_planner', 'vision'):
        subprocess.run([sys.executable, str(ROOT/'restart_component.py'), component], check=True, timeout=35)
    vision_pid = int((ROOT/'vision.pid').read_text())
    for prefix in ('local', 'global'):
        client = n.create_client(ClearEntireCostmap, '/'+prefix+'_costmap/clear_entirely_'+prefix+'_costmap')
        assert client.wait_for_service(timeout_sec=3)
        future = client.call_async(ClearEntireCostmap.Request())
        rclpy.spin_until_future_complete(n, future, timeout_sec=5)
        assert future.done()
        n.destroy_client(client)
    n.spin(3.)
    report['setup_position'] = n.state['native']['position']
    report['setup_direction'] = n.state['native']['direction']
    report['direction_matches_recording'] = float(np.dot(n.state['native']['direction'], forward)) > .98
    assert report['direction_matches_recording'], 'Reproduced orientation differs'
    n.change_query('traffic light')
    assert n.spin(20., lambda: len(bearings) >= 2), 'No positive mask in reproduced view'
    report['first_bearing'] = bearings[0]
    initial = np.asarray(n.state['native']['position'][:2])
    unit = np.array([np.cos(bearings[0]['bearing']), np.sin(bearings[0]['bearing'])])
    n.set_bool('/wildos_cmd_vel_to_control', 'enable', True)
    n.set_bool('/wildos_graph_path_bridge', 'follow_path', True)
    started = time.monotonic()
    last_log = 0.
    while time.monotonic()-started < 120:
        n.spin(.1); n.watch()
        if time.monotonic()-last_log > 5:
            current = np.asarray(n.state['native']['position'][:2])
            n.event('approach_progress', position=current.tolist(),
                    advance_in_initial_bearing=float((current-initial)@unit),
                    speed=n.state['native']['speed'], status=n.last_status)
            last_log = time.monotonic()
        if n.last_status and n.last_status.get('status') == 'object_found':
            break
    current = np.asarray(n.state['native']['position'][:2])
    report.update(advance_in_initial_bearing=float((current-initial)@unit),
                  mask_count=len(bearings), sensor_pairs=saved_pairs,
                  latest_status=n.last_status, confirmation=n.confirmation,
                  final_position=n.state['native']['position'])
except BaseException as exc:
    report['error'] = repr(exc)
finally:
    try:
        n.stop()
    finally:
        b.control.pause(); b.disconnect()
        report['vision_pid_retained'] = Path('/proc', str(vision_pid)).exists()
        report['bearings'] = bearings
        (out/'focused_report.json').write_text(json.dumps(report, indent=2))
        n.video.release(); n.log.close(); n.native_log.close(); n.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
print(json.dumps({k:v for k,v in report.items() if k != 'bearings'}, indent=2), flush=True)
