"""One loaded Town map: observe station, search house, return via memory."""
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from beamngpy import BeamNGpy
import cv2
import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
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
from wildos_nav2_adapter.arrival_criteria import (
    STANDOFF_ARRIVAL_TOLERANCE_M, arrival_status_matches,
    mask_matches_confirmation,
)
from wildos_nav2_adapter.test_progress import ManeuverProgress

session = Path(os.environ.get('WILDOS_TEST_SESSION_DIR', '/home/ly/full_town_memory_20261002'))
out = session / (sys.argv[1] if len(sys.argv) > 1 else 'attempt2')
out.mkdir(exist_ok=True)
calibration = {'speed_tuning_performed': False}
vision_pid = int(session.joinpath('vision.pid').read_text())
b = BeamNGpy('127.0.0.1', 25252)
b.open(launch=False)
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)


class Harness(Node):
    def __init__(self):
        super().__init__('full_town_memory_test')
        self.phase = 'station_observation'
        self.query = 'gas station'
        self.query_started_ns = 0
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
        self.arrival_statuses = []
        self.waypoint = None
        self.confirmation = None
        self.query_pub = self.create_publisher(String, '/spot1/object_search/query', 10)
        self.cmd_pub = self.create_publisher(Twist, '/wildos/cmd_vel', 1)
        self.create_subscription(String, '/wildos/native_control_status', self.native, 10)
        self.create_subscription(String, '/wildos/plan_status', self.status, 10)
        self.create_subscription(String, '/spot1/remembered_object_candidates', self.candidates, 10)
        self.create_subscription(String, '/wildos/memory_selection', self.selection, 10)
        self.create_subscription(PoseStamped, '/spot1/imgnav_waypoint', self.goal, 10)
        self.create_subscription(String, '/spot1/object_confirmation', self.confirmed, 10)
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
        if value.get('status') == 'object_found':
            self.arrival_statuses.append({'status': value, 'received_at': time.monotonic()})
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

    def confirmed(self, msg):
        data = json.loads(msg.data)
        now = self.get_clock().now().nanoseconds
        stamp = data.get('observation_stamp_ns', 0)
        if (data.get('query') != self.query or not isinstance(stamp, int)
                or stamp < self.query_started_ns or not 0 <= now - stamp <= 5 * 10**9):
            return
        self.confirmation = dict(data, received_at=time.monotonic())
        self.event('object_confirmation', payload=data)

    def path(self, msg):
        points = [xy(p.pose) for p in msg.poses]
        self.state['path'] = points
        self.event('planned_path', poses=len(points), points=points, headings=[yaw(p.pose) for p in msg.poses],
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
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if (stamp_ns < self.query_started_ns or
                not 0 <= self.get_clock().now().nanoseconds - stamp_ns <= 5 * 10**9):
            return
        old = self.masks.get(msg.query, {})
        if stamp_ns <= old.get('observation_stamp_ns', 0):
            return
        self.masks[msg.query] = {'at': time.monotonic(), 'pixels': pixels,
                                'count': old.get('count', 0)+1, 'observation_stamp_ns': stamp_ns}
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
        rclpy.spin_until_future_complete(self, future, timeout_sec=10)
        if not future.done():
            self.destroy_client(client)
            raise RuntimeError('Parameter update timeout: ' + remote + '/' + name)
        results = future.result().results
        self.destroy_client(client)
        if not all(r.successful for r in results):
            raise RuntimeError('Parameter update rejected: ' + name + ': ' + str([r.reason for r in results]))

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
        self.query_started_ns = self.get_clock().now().nanoseconds
        self.waypoint = None
        self.confirmation = None
        self.masks.pop(query, None)
        self.state.pop('selection', None)
        self.event('query_sent')
        self.query_pub.publish(String(data=query))

    def arrival_evidence(self, query, minimum_mask_count):
        confirmation = self.confirmation
        mask = self.masks.get(query, {})
        now = time.monotonic()
        if (not confirmation or confirmation.get('query') != query
                or confirmation.get('search_confirmed') is not True
                or now - confirmation.get('received_at', 0.) >= 5.
                or mask.get('count', 0) < minimum_mask_count
                or mask.get('pixels', 0) <= 0 or now - mask.get('at', 0.) >= 5.
                or not mask_matches_confirmation(query, mask.get('observation_stamp_ns'), confirmation)):
            return None
        for item in reversed(self.arrival_statuses):
            if (now - item['received_at'] < 5.
                    and arrival_status_matches(item['status'], query, confirmation,
                                               STANDOFF_ARRIVAL_TOLERANCE_M)):
                return item
        return None

    def watch(self):
        if time.monotonic() - self.state.get('native_at', 0) > .5:
            raise RuntimeError('Native vehicle state stalled')
        if not Path('/proc', str(vision_pid)).exists():
            raise RuntimeError('Vision process exited')


n = Harness()
bag = None
report = {'passed': False, 'vision_pid': vision_pid, 'map_reload_count': 0,
          'vehicle_teleports_during_sequence': 0, 'search_query': 'house', 'minimum_departure_m': None, 'calibration': {k:v for k,v in calibration.items() if k != 'samples'}}
try:
    n.set_bool('/wildos_graph_path_bridge', 'follow_path', False)
    n.set_bool('/wildos_cmd_vel_to_control', 'enable', False)
    if os.environ.get('WILDOS_REQUIRE_BODY_COLLISION_DISABLED') == '1':
        profile = json.loads(b.control.queue_lua_command("""
local veh=scenetree.findObject('ego')
local out=extensions.wildosBodyCollision.status(veh:getID())
out.mission=getMissionFilename()
out.collisionMasks={}
for _,node in pairs(core_vehicle_manager.getVehicleData(veh:getID()).vdata.nodes) do
  local mask=tostring(veh:getInitialNodeCollision(node.cid))
  out.collisionMasks[mask]=(out.collisionMasks[mask] or 0)+1
end
out.cameras=extensions.tech_sensors.getActiveCameraSensors()
out.lidars=extensions.tech_sensors.getActiveLidarSensors()
return jsonEncode(out)
""", response=True))
        report['collision_profile_before_sequence'] = profile
        if not profile.get('liveVerified') or profile['collisionMasks'].get('0', 0) < profile['liveCounts']['bodyNodeCount']:
            raise RuntimeError('Body collision profile is not physically disabled')
        if not profile['cameras'] or not profile['lidars']:
            raise RuntimeError('Camera or LiDAR missing after disabling body collisions')
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
              '/spot1/model_visualization', '/spot1/within_range_geofrontiers', '/spot1/score_rings',
              '/spot1/nav_graph_viz', '/spot1/graphnav_planner/frontier_scores',
              '/spot1/remembered_object_feedback', '/wildos/memory_selection', '/spot1/imgnav_waypoint',
              '/spot1/odom', '/spot1/nav_graph', '/spot1/scored_nav_graph', '/wildos/planned_path',
              '/spot1/graphnav_planner/path', '/wildos/full_planned_path',
              '/spot1/graphnav_planner/route_status', '/spot1/object_localization',
              '/spot1/object_confirmation', '/wildos/viz/frontier_traversability',
              '/wildos/control_status', '/wildos/execution_feedback', '/spot1/execution_anchor_graph', '/spot1/traversal_feedback',
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
    if not n.spin(10, lambda: any('rosbag2_recorder' in info.node_name
            for info in n.get_subscriptions_info_by_topic('/spot1/object_search/query'))):
        raise RuntimeError('Recorder did not subscribe to query topic')
    n.spin(.5)
    n.change_query('gas station')
    last_replay = 0.
    deadline = time.monotonic() + 180
    while time.monotonic() < deadline and not (n.views.get('gas station') and n.confirmation):
        n.spin(.2)
        n.watch()
        if time.monotonic() - last_replay > 10:
            n.query_pub.publish(String(data='gas station'))
            last_replay = time.monotonic()
    report['station_views_before_departure'] = n.views.get('gas station', [])
    if not report['station_views_before_departure'] or not n.confirmation:
        raise RuntimeError('Station was not visually confirmed and remembered at spawn')
    n.event('station_memory_established', views=report['station_views_before_departure'])
    n.set_bool('/wildos_cmd_vel_to_control', 'enable', True)
    n.set_bool('/wildos_graph_path_bridge', 'follow_path', True)
    n.phase = 'house_search'
    n.change_query('house')
    search_started = time.monotonic()
    house_found = False
    maximum_departure = 0.
    last_progress = search_started
    maneuver_progress = ManeuverProgress()
    previous = born
    best_departure = 0.
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
                    mask_age=time.monotonic()-n.masks.get('house', {}).get('at', 0))
            last_progress_report = time.monotonic()
        if maximum_departure > best_departure + .3:
            last_progress, best_departure = time.monotonic(), maximum_departure
        if 'odom' in n.state and maneuver_progress.observe(xy(n.state['odom'].pose.pose), n.last_status):
            last_progress = time.monotonic()
        target = n.waypoint
        arrival = n.arrival_evidence('house', minimum_mask_count=3)
        if arrival:
            house_found = True
            confirmation = n.confirmation
            report['house_confirmation'] = dict(
                mask=n.masks['house'], confirmation=confirmation,
                confirmed_standoff=confirmation['standoff'], vehicle_position=point,
                navigation_position=xy(n.state['odom'].pose.pose),
                standoff_distance=arrival['status']['standoff_distance_m'],
                arrival_status=arrival['status'])
            break
        if time.monotonic() - last_progress > 65:
            n.event('search_no_progress', last_status=n.last_status)
            break
    n.stop()
    report.update(house_found=house_found, search_elapsed=time.monotonic()-search_started,
                  maximum_departure=maximum_departure, house_end=n.state['native']['position'])
    n.event('house_search_finished', found=house_found, maximum_departure=maximum_departure)
    if not house_found:
        raise RuntimeError('House was not confirmed; full multi-goal sequence cannot proceed')
    n.set_bool('/wildos_cmd_vel_to_control', 'enable', True)
    n.set_bool('/wildos_graph_path_bridge', 'follow_path', True)
    n.phase = 'station_return'
    n.change_query('gas station')
    switched = time.monotonic()
    returned = False
    first_memory_selection = first_memory_path = None
    last_progress = time.monotonic()
    maneuver_progress = ManeuverProgress()
    previous = n.state['native']['position'][:2]
    return_started_distance = math.dist(previous, born)
    best_return_distance = math.dist(previous, born)
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
        return_distance = math.dist(point, born)
        if return_distance < best_return_distance - .3:
            last_progress, best_return_distance = time.monotonic(), return_distance
        if 'odom' in n.state and maneuver_progress.observe(xy(n.state['odom'].pose.pose), n.last_status):
            last_progress = time.monotonic()
        if n.state.get('selection', {}).get('status') == 'memory_candidate_selected' and first_memory_selection is None:
            first_memory_selection = time.monotonic() - switched
        if n.last_status and n.last_status.get('source') == 'memory_via_graphnav' and n.last_status.get('status') in ('planned', 'following') and first_memory_path is None:
            first_memory_path = time.monotonic() - switched
        arrival = n.arrival_evidence('gas station', minimum_mask_count=2)
        if arrival:
            status = arrival['status']
            if not status.get('memory_view_id'):
                continue
            memory_delta = status.get('memory_region_distance_m')
            memory_tolerance = status.get('memory_region_tolerance_m')
            if (not isinstance(memory_delta, (int, float))
                    or not isinstance(memory_tolerance, (int, float))
                    or not math.isfinite(memory_delta) or not math.isfinite(memory_tolerance)
                    or memory_delta > memory_tolerance):
                continue
            returned = True
            report['return_confirmation'] = n.confirmation
            report['return_arrival_status'] = status
            report['return_memory_view_id'] = status['memory_view_id']
            report['return_memory_region_match_m'] = memory_delta
            report['return_memory_region_tolerance_m'] = memory_tolerance
            break
        if time.monotonic() - last_progress > 65:
            n.event('return_no_progress', last_status=n.last_status)
            break
    n.stop()
    report.update(returned=returned, return_elapsed=time.monotonic()-switched,
                  minimum_departure_m=None,
                  memory_selection_latency=first_memory_selection, memory_path_latency=first_memory_path,
                  memory_navigation_exercised=first_memory_path is not None,
                  final_position=n.state['native']['position'],
                  vision_pid_retained=Path('/proc', str(vision_pid)).exists(),
                  passed=(house_found and returned and first_memory_path is not None
                          and Path('/proc', str(vision_pid)).exists()))
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
        if rclpy.ok(): rclpy.shutdown()
print(json.dumps({k:v for k,v in report.items() if k != 'events'}, indent=2), flush=True)
raise SystemExit(0 if report['passed'] else 1)
