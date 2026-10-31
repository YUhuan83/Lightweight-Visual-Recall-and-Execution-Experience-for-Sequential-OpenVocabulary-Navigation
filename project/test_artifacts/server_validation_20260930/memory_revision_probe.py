"""Live GraphNav/CarLike integration with fixtures, paused and follow_path=false."""
import json
import math
from pathlib import Path
import time

from beamngpy import BeamNGpy
from beamng_msgs.msg import VehicleControl
import rclpy
from std_msgs.msg import String
from wildos_nav2_adapter.graph_path_bridge import GraphPathBridge, xy, yaw

beam = BeamNGpy('127.0.0.1', 25252)
beam.open(launch=False)
beam.control.pause()
ego = beam.vehicles.get_current()['ego']
ego.connect(beam)
ego.sensors.poll()
before = dict(ego.state)
report = {'scope': 'synthetic candidate fixtures, live GraphNav and CarLike, no motion',
          'wildos_original_pid_alive': Path('/proc/1575639').exists(), 'before': before,
          'cases': [], 'controls': []}
rclpy.init(args=['--ros-args', '-r', '__node:=wildos_memory_revision_probe', '-p', 'follow_path:=false'])


class Probe(GraphPathBridge):
    def __init__(self):
        self.events = []
        super().__init__()
        self.diagnostic = None
        self.create_subscription(String, '/wildos/memory_selection', self.selection, 10)
        self.create_subscription(VehicleControl, '/control', self.control, 10)

    def status(self, state, **details):
        self.events.append({'t': time.monotonic(), 'status': state, **details})
        super().status(state, **details)

    def candidate_failed(self, reason):
        self.events.append({'t': time.monotonic(), 'candidate_failed': reason,
                            'view': self.memory_active['id'] if self.memory_active else None,
                            'candidate': self.candidate_index})
        super().candidate_failed(reason)

    def record_memory_path(self, path, points, headings):
        self.events.append({'t': time.monotonic(), 'validated_path': True,
                            'view': self.memory_active['id'], 'poses': len(points),
                            'length': sum(math.dist(a, b) for a, b in zip(points, points[1:])),
                            'arrival_heading': headings[-1]})
        super().record_memory_path(path, points, headings)

    def selection(self, msg):
        value = json.loads(msg.data)
        if value.get('query') == self.query:
            self.diagnostic = value

    def control(self, msg):
        value = {'throttle': msg.throttle, 'brake': msg.brake, 'gear': msg.gear}
        if not report['controls'] or report['controls'][-1] != value:
            report['controls'].append(value)


node = Probe()


def spin(seconds, done=lambda: False):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.1)
        if done():
            return True
    return False


def view(name, point, heading):
    return {'id': name, 'frame': 'spot1/odom', 'position': [*point, 52.154], 'heading': heading}


try:
    if not spin(15, lambda: node.input_freshness_issue() is None):
        raise RuntimeError('No fresh live odom/costmap')
    start, heading = xy(node.odom.pose.pose), yaw(node.odom.pose.pose)
    report['live_start'] = [*start, heading]
    direction = math.cos(heading), math.sin(heading)
    forward = lambda d: (start[0] + direction[0] * d, start[1] + direction[1] * d)
    fixtures = [
        ('remembered_and_current_fixture', [
            view('historical_tree_position', (740.5373794974876, -14.956937346942368), -.48737),
            view('current_position_fixture', start, heading)]),
        ('heading_and_cost_fixture', [
            view('forward_4_5m', forward(4.5), heading + math.pi),
            view('forward_3m', forward(3.0), heading + math.pi)]),
        ('historical_position_only', [
            view('historical_tree_position', (740.5373794974876, -14.956937346942368), -.48737)]),
    ]
    # Pick an actual occupied cell from the live costmap. Do not infer that a
    # previously obstructed historical position is still obstructed today.
    grid = node.grid
    q = grid.info.origin.orientation
    grid_yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
    occupied = []
    for index, value in enumerate(grid.data):
        if value < 99:
            continue
        gx = (index % grid.info.width + .5) * grid.info.resolution
        gy = (index // grid.info.width + .5) * grid.info.resolution
        point = (grid.info.origin.position.x + math.cos(grid_yaw)*gx - math.sin(grid_yaw)*gy,
                 grid.info.origin.position.y + math.sin(grid_yaw)*gx + math.cos(grid_yaw)*gy)
        distance = math.dist(point, start)
        if 3 < distance < 20:
            occupied.append((abs(distance - 8), point, value))
    if occupied:
        _, blocked, value = min(occupied)
        report['occupied_fixture'] = {'position': blocked, 'cost': value}
        fixtures.extend([
            ('blocked_then_valid_fixture', [view('occupied', blocked, heading),
                                           view('valid_forward', forward(3), heading)]),
            ('all_blocked_fixture', [view('occupied', blocked, heading)])])
    else:
        report['blocked_case_skipped'] = 'No live occupied cell within 20 metres'
    for label, views in fixtures:
        node.diagnostic = None
        node.events = []
        query = '__memory_probe_' + label
        started = time.monotonic()
        node.on_query(String(data=query))  # Local callback: leaves WildOS query unchanged.
        node.on_memory_candidates(String(data=json.dumps({'query': query, 'views': views})))
        complete = spin(40, lambda: node.diagnostic is not None)
        spin(2)
        report['cases'].append({'label': label, 'fixtures': views, 'completed': complete,
                                'elapsed': time.monotonic() - started,
                                'selection': node.diagnostic, 'status': node.last_status,
                                'events': node.events})
        print(json.dumps({k: v for k, v in report['cases'][-1].items() if k not in ('events', 'fixtures')}), flush=True)
        selection = node.diagnostic or {}
        expected = {'remembered_and_current_fixture': 'current_position_fixture',
                    'heading_and_cost_fixture': 'forward_3m',
                    'historical_position_only': 'historical_tree_position',
                    'blocked_then_valid_fixture': 'valid_forward'}
        passed = complete and (selection.get('view_id') == expected[label]
                               if label in expected else selection.get('status') == 'memory_unreachable')
        if label == 'blocked_then_valid_fixture':
            passed = passed and 'occupied' in selection.get('rejected', [])
        if label == 'all_blocked_fixture':
            passed = passed and node.memory_goal is None and not node.query_reset_pending
        report['cases'][-1]['passed'] = passed
    node.on_query(String(data='__memory_probe_finished'))
    spin(2)
finally:
    node.reset()
    node.destroy_node()
    rclpy.shutdown()
    beam.control.pause()
    ego.sensors.poll()
    report['after'] = dict(ego.state)
    report['movement'] = math.dist(report['before']['pos'], report['after']['pos'])
    report['follow_path_enabled'] = False
    beam.disconnect()
    Path('/home/ly/navigation_validation_20260930_logs/memory_revision_probe.json').write_text(json.dumps(report, indent=2))
    print('Paused planning probe displacement:', report['movement'], flush=True)
if not all(case['passed'] for case in report['cases']):
    raise RuntimeError('A memory selection integration case failed')
if report['movement'] != 0 or any(c['throttle'] != 0 or c['brake'] != 1 for c in report['controls']):
    raise RuntimeError('Planning-only probe changed vehicle movement/control')
