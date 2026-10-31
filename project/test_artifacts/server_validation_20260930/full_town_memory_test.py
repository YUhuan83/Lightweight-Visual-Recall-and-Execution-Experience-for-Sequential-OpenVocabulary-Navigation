"""One loaded Town map: observe station, search trash can, return via memory."""
from collections import deque
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import time

from beamngpy import BeamNGpy
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid, Path as Route
from object_search_msgs.msg import ObjectMaskWithTf
from sensor_msgs.msg import Image
from std_msgs.msg import String
from wildos_nav2_adapter.graph_path_bridge import xy, yaw

session = Path('/home/ly/full_town_memory_20260930')
out = session / (sys.argv[1] if len(sys.argv) > 1 else 'attempt2')
out.mkdir(exist_ok=True)
calibration = json.loads(session.joinpath('control_calibration.json').read_text())
vision_pid = int(session.joinpath('vision.pid').read_text())
b = BeamNGpy('127.0.0.1', 25252)
b.open(launch=False)
rclpy.init()


class Harness(Node):
    def __init__(self):
        super().__init__('full_town_memory_test')
        self.phase = 'station_observation'
        self.query = 'gas station'
        self.state = {}
        self.events = []
        self.masks = {}
        self.views = {}
        self.frames = deque(maxlen=12)
        self.video = cv2.VideoWriter(str(out / 'front_camera.mp4'), cv2.VideoWriter_fourcc(*'mp4v'), 5, (960, 640))
        self.log = out.joinpath('events.jsonl').open('w')
        self.native_log = out.joinpath('native_samples.jsonl').open('w')
        self.start = time.monotonic()
        self.last_native_write = 0.
        self.last_status = None
        self.waypoint = None
        self.query_pub = self.create_publisher(String, '/spot1/object_search/query', 10)
        self.cmd_pub = self.create_publisher(Twist, '/wildos/cmd_vel', 1)
        self.create_subscription(String, '/wildos/native_control_status', self.native, 10)
        self.create_subscription(String, '/wildos/plan_status', self.status, 10)
        self.create_subscription(String, '/spot1/remembered_object_candidates', self.candidates, 10)
        self.create_subscription(String, '/wildos/memory_selection', self.selection, 10)
        self.create_subscription(PoseStamped, '/spot1/imgnav_waypoint', self.goal, 10)
        self.create_subscription(ObjectMaskWithTf, '/spot1/object_mask', self.mask, 10)
        self.create_subscription(Image, '/spot1/realsense/front/color/image_raw', self.camera, 1)
        self.create_subscription(Route, '/wildos/planned_path', self.path, 10)
        self.create_subscription(Odometry, '/spot1/odom', lambda m: self.state.update(odom=m), qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: self.state.update(grid=m), 10)

    def event(self, kind, **details):
        value = {'elapsed': time.monotonic() - self.start, 'phase': self.phase, 'query': self.query,
                 'kind': kind, **details}
        self.events.append(value)
        self.log.write(json.dumps(value) + '\n')
        self.log.flush()
        print(json.dumps(value), flush=True)

    def native(self, msg):
        self.state['native'] = json.loads(msg.data)
        self.state['native_at'] = time.monotonic()
        if self.state['native_at'] - self.last_native_write >= .1:
            self.native_log.write(json.dumps({'phase': self.phase, **self.state['native']}) + '\n')
            self.last_native_write = self.state['native_at']

    def status(self, msg):
        value = json.loads(msg.data)
        if value != self.last_status:
            self.event('plan_status', **value)
            self.last_status = value

    def candidates(self, msg):
        data = json.loads(msg.data)
        self.views[data['query']] = data['views']
        self.event('memory_candidates', payload=data)

    def selection(self, msg):
        data = json.loads(msg.data)
        self.state['selection'] = data
        self.event('memory_selection', payload=data)

    def goal(self, msg):
        self.waypoint = {'point': xy(msg.pose), 'at': time.monotonic(), 'query': self.query}
        self.event('object_standoff', point=self.waypoint['point'])

    def path(self, msg):
        points = [xy(p.pose) for p in msg.poses]
        self.state['path'] = points
        self.event('planned_path', poses=len(points), points=points,
                   length=sum(math.dist(a, c) for a, c in zip(points, points[1:])))

    def camera(self, msg):
        data = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width*3]
        frame = cv2.cvtColor(data.reshape(msg.height, msg.width, 3), cv2.COLOR_RGB2BGR)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        self.frames.append((stamp, frame.copy()))
        self.video.write(frame)
        cv2.imwrite(str(out / (self.phase + '_latest.png')), frame)

    def mask(self, msg):
        pixels = int(sum(msg.object_mask.data))
        if msg.query != self.query:
            self.event('old_mask', mask_query=msg.query)
            return
        old = self.masks.get(msg.query, {})
        self.masks[msg.query] = {'at': time.monotonic(), 'pixels': pixels,
                                'count': old.get('count', 0)+1}
        if self.frames:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            _, frame = min(self.frames, key=lambda p: abs(p[0]-stamp))
            dims = [d.size for d in msg.object_mask.layout.dim]
            mask = np.array(msg.object_mask.data, dtype=np.uint8).reshape(dims)[0, 0] > 0
            overlay = frame.copy()
            overlay[mask] = (.5 * overlay[mask] + .5 * np.array([0, 255, 0])).astype(np.uint8)
            cv2.imwrite(str(out / (self.phase + '_mask.png')), overlay)

    def set_bool(self, remote, name, value):
        client = self.create_client(SetParameters, remote + '/set_parameters')
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError('Missing parameter service: ' + remote)
        parameter = Parameter(name=name, value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value))
        future = client.call_async(SetParameters.Request(parameters=[parameter]))
        rclpy.spin_until_future_complete(self, future, timeout_sec=3)
        if not future.done() or not all(r.successful for r in future.result().results):
            raise RuntimeError('Parameter update failed: ' + name)

    def spin(self, seconds, done=lambda: False):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.05)
            if done():
                return True
        return False

    def stop(self):
        self.cmd_pub.publish(Twist())
        self.set_bool('/wildos_graph_path_bridge', 'follow_path', False)
        self.set_bool('/wildos_cmd_vel_to_control', 'enable', False)
        return self.spin(3, lambda: self.state.get('native', {}).get('speed', 99) < .1)

    def change_query(self, query):
        self.query = query
        self.waypoint = None
        self.masks.pop(query, None)
        self.state.pop('selection', None)
        self.event('query_sent')
        self.query_pub.publish(String(data=query))

    def watch(self):
        if time.monotonic() - self.state.get('native_at', 0) > .5:
            raise RuntimeError('Native vehicle state stalled')
        if not Path('/proc', str(vision_pid)).exists():
            raise RuntimeError('Vision process exited')


n = Harness()
bag = None
report = {'passed': False, 'vision_pid': vision_pid, 'map_reload_count': 0,
          'vehicle_teleports_during_sequence': 0, 'calibration': {k:v for k,v in calibration.items() if k != 'samples'}}
try:
    n.set_bool('/wildos_graph_path_bridge', 'follow_path', False)
    n.set_bool('/wildos_cmd_vel_to_control', 'enable', False)
    # Prepare the next attempt at the requested spawn. The map and vision
    # process stay loaded; there are no teleports between the two search goals.
    b.control.pause()
    b.vehicles.teleport('ego', (734.294, -11.946, 52.154),
                        (0., 0., .7658087977066911, .6430683364581344), reset=True)
    b.control.resume()
    n.spin(2.)  # Drain pre-teleport telemetry before checking the spawn.
    if not n.spin(10, lambda: bool(n.frames) and 'native' in n.state):
        raise RuntimeError('No native state or camera')
    born = n.state['native']['position'][:2]
    report['start_position'] = born
    if math.dist(born, (734.294, -11.946)) > 1:
        raise RuntimeError('Outside required gas-station spawn')
    topics = ['/spot1/object_search/query', '/spot1/object_mask', '/spot1/remembered_object_candidates',
              '/spot1/remembered_object_feedback', '/wildos/memory_selection', '/spot1/imgnav_waypoint',
              '/spot1/odom', '/spot1/nav_graph', '/spot1/scored_nav_graph', '/wildos/planned_path',
              '/wildos/plan_status', '/wildos/cmd_vel', '/wildos/native_control',
              '/wildos/native_control_status', '/global_costmap/costmap', '/tf', '/tf_static',
              '/received_global_plan', '/lookahead_point', '/lookahead_collision_arc',
              '/global_costmap/costmap_raw', '/local_costmap/costmap', '/local_costmap/costmap_raw',
              '/vehicles/ego/sensors/state', '/vehicles/ego/sensors/front_lidar',
              '/spot1/ouster/front/points_filtered', '/wildos/points', '/wildos/clearing_points',
              '/rosout']
    with out.joinpath('rosbag.log').open('ab') as f:
        bag = subprocess.Popen(['ros2', 'bag', 'record', '-o', str(out / 'rosbag'), *topics],
                               stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
    n.change_query('gas station')
    last_replay = 0.
    deadline = time.monotonic() + 180
    while time.monotonic() < deadline and not n.views.get('gas station'):
        n.spin(.2)
        n.watch()
        if time.monotonic() - last_replay > 10:
            n.query_pub.publish(String(data='gas station'))
            last_replay = time.monotonic()
    report['station_views_before_departure'] = n.views.get('gas station', [])
    if not report['station_views_before_departure']:
        raise RuntimeError('Station was not visually confirmed and remembered at spawn')
    n.event('station_memory_established', views=report['station_views_before_departure'])
    n.phase = 'trash_can_search'
    n.change_query('trash can')
    n.spin(3)
    n.set_bool('/wildos_cmd_vel_to_control', 'enable', True)
    n.set_bool('/wildos_graph_path_bridge', 'follow_path', True)
    search_started = time.monotonic()
    trash_found = False
    maximum_departure = 0.
    last_progress = search_started
    previous = born
    last_progress_report = 0.
    deadline = search_started + 420
    while time.monotonic() < deadline:
        n.spin(.1)
        n.watch()
        point = n.state['native']['position'][:2]
        maximum_departure = max(maximum_departure, math.dist(point, born))
        if time.monotonic() - last_progress_report > 10:
            n.event('search_progress', position=point, departure=maximum_departure,
                    speed=n.state['native']['speed'], status=n.last_status,
                    mask_age=time.monotonic()-n.masks.get('trash can', {}).get('at', 0))
            last_progress_report = time.monotonic()
        if math.dist(point, previous) > .3:
            last_progress, previous = time.monotonic(), point
        mask = n.masks.get('trash can', {})
        target = n.waypoint
        if (target and target['query'] == 'trash can' and time.monotonic()-mask.get('at', 0) < 5
                and mask.get('count',0) >= 3 and mask.get('pixels',0) > 0 and maximum_departure > 5):
            # Search success means a repeatable visual detection corroborated
            # by the triangulator's confirmed object waypoint. Reaching an
            # arbitrary five-metre standoff is a separate approach criterion.
            trash_found = True
            report['trash_confirmation'] = dict(mask=mask, confirmed_standoff=target['point'],
                                                vehicle_position=point,
                                                standoff_distance=math.dist(point,target['point']))
            break
        if time.monotonic() - last_progress > 65:
            n.event('search_no_progress', last_status=n.last_status)
            break
    n.stop()
    report.update(trash_found=trash_found, search_elapsed=time.monotonic()-search_started,
                  maximum_departure=maximum_departure, trash_end=n.state['native']['position'])
    n.event('trash_search_finished', found=trash_found, maximum_departure=maximum_departure)
    n.phase = 'station_return'
    n.change_query('gas station')
    switched = time.monotonic()
    n.spin(3)
    n.set_bool('/wildos_cmd_vel_to_control', 'enable', True)
    n.set_bool('/wildos_graph_path_bridge', 'follow_path', True)
    returned = False
    first_memory_selection = first_memory_path = None
    last_progress = time.monotonic()
    previous = n.state['native']['position'][:2]
    last_progress_report = 0.
    deadline = time.monotonic() + 420
    while time.monotonic() < deadline:
        n.spin(.1)
        n.watch()
        point = n.state['native']['position'][:2]
        if time.monotonic() - last_progress_report > 10:
            n.event('return_progress', position=point, speed=n.state['native']['speed'],
                    status=n.last_status, selection=n.state.get('selection'))
            last_progress_report = time.monotonic()
        if math.dist(point, previous) > .3:
            last_progress, previous = time.monotonic(), point
        if n.state.get('selection', {}).get('status') == 'memory_candidate_selected' and first_memory_selection is None:
            first_memory_selection = time.monotonic() - switched
        if n.last_status and n.last_status.get('source') == 'memory_via_graphnav' and n.last_status.get('status') in ('planned', 'following') and first_memory_path is None:
            first_memory_path = time.monotonic() - switched
        goal = n.state.get('selection', {}).get('goal')
        mask = n.masks.get('gas station', {})
        if (goal and maximum_departure > 5 and math.dist(point, goal) < 1.5 and
                time.monotonic()-mask.get('at', 0) < 5):
            returned = True
            break
        if time.monotonic() - last_progress > 65:
            n.event('return_no_progress', last_status=n.last_status)
            break
    n.stop()
    report.update(returned=returned, return_elapsed=time.monotonic()-switched,
                  memory_selection_latency=first_memory_selection, memory_path_latency=first_memory_path,
                  final_position=n.state['native']['position'],
                  vision_pid_retained=Path('/proc', str(vision_pid)).exists(),
                  passed=trash_found and returned and Path('/proc', str(vision_pid)).exists())
    n.event('full_test_finished', **{k:v for k,v in report.items() if k not in ('calibration','station_views_before_departure')})
except BaseException as exc:
    report['error'] = repr(exc)
    n.event('test_error', error=repr(exc))
finally:
    b.control.pause()
    try:
        n.stop()
    finally:
        b.disconnect()
        if bag:
            bag.send_signal(signal.SIGINT)
            try:
                bag.wait(timeout=10)
            except subprocess.TimeoutExpired:
                bag.terminate()
        report['events'] = n.events
        report['simulation_paused'] = True
        out.joinpath('full_test.json').write_text(json.dumps(report, indent=2))
        n.video.release()
        n.log.close()
        n.native_log.close()
        n.destroy_node()
        rclpy.shutdown()
print(json.dumps({k:v for k,v in report.items() if k != 'events'}, indent=2), flush=True)
