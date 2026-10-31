"""Validate speed and prompt brakes with native measurements before the search."""
import json
import math
from pathlib import Path
import time
from beamngpy import BeamNGpy
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from wildos_nav2_adapter.graph_route import footprint_blockers

out = Path('/home/ly/full_town_memory_20260930')
b = BeamNGpy('127.0.0.1', 25252)
b.open(launch=False)
v = b.vehicles.get_current()['ego']
v.connect(b)
rclpy.init()
n = Node('full_test_control_calibration')
s = {}
samples = []
def receive(msg):
    s['native'] = json.loads(msg.data)
    s['received_at'] = time.monotonic()
    samples.append(s['native'])
n.create_subscription(String, '/wildos/native_control_status', receive, 10)
n.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: s.update(grid=m), 10)
pub = n.create_publisher(Twist, '/wildos/cmd_vel', 1)
def spin(seconds, done=lambda: False):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=.02)
        if done():
            return True
    return False
def param(remote, name, enabled):
    client = n.create_client(SetParameters, remote + '/set_parameters')
    if not client.wait_for_service(timeout_sec=3):
        raise RuntimeError('Missing parameter service: ' + remote)
    p = Parameter(name=name, value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=enabled))
    f = client.call_async(SetParameters.Request(parameters=[p]))
    rclpy.spin_until_future_complete(n, f, timeout_sec=3)
    if not f.done() or not all(r.successful for r in f.result().results):
        raise RuntimeError('Parameter change failed: ' + name)

report = {'passed': False, 'scope': 'native measured straight start/stop, map retained'}
try:
    if not spin(5, lambda: 'native' in s and 'grid' in s):
        raise RuntimeError('Missing native status or costmap')
    v.sensors.poll()
    initial = dict(v.state)
    start = initial['pos'][:2]
    heading = math.atan2(initial['dir'][1], initial['dir'][0])
    alternatives = [heading + offset for offset in (0, math.pi/2, -math.pi/2, math.pi)]
    free_heading = next((angle for angle in alternatives if all(not footprint_blockers(
        s['grid'], (start[0] + math.cos(angle)*i*.25, start[1] + math.sin(angle)*i*.25), angle)
        for i in range(13))), None)
    if free_heading is None:
        raise RuntimeError('No free calibration approach within 3 metres')
    if abs(math.atan2(math.sin(free_heading-heading), math.cos(free_heading-heading))) > .1:
        b.control.pause()
        angle = -math.pi/2 - free_heading
        b.vehicles.teleport('ego', (734.294, -11.946, 52.154),
                            (0., 0., math.sin(angle/2), math.cos(angle/2)), reset=True)
    report['calibration_heading'] = free_heading
    b.control.resume()
    param('/wildos_graph_path_bridge', 'follow_path', False)
    param('/wildos_cmd_vel_to_control', 'enable', True)
    drive_start = time.monotonic()
    command = Twist()
    command.linear.x = 1.2
    while time.monotonic() - drive_start < 12:
        pub.publish(command)
        spin(.08)
        current = s['native']
        if time.monotonic() - s['received_at'] > .4:
            raise RuntimeError('Native state stream stalled')
        if current['speed'] > 1.8:
            raise RuntimeError('Native overspeed')
        if math.dist(start, current['position'][:2]) >= 2:
            break
    stop_sent = time.time()
    pub.publish(Twist())
    param('/wildos_cmd_vel_to_control', 'enable', False)
    stopped = spin(4, lambda: s['native']['speed'] < .1)
    after = s['native']
    driven = [p for p in samples if p['wall_time'] < stop_sent and p['speed'] > .05]
    brake_samples = [p for p in samples if p['wall_time'] >= stop_sent and
                     p['actual'].get('throttle') == 0 and p['actual'].get('brake', 0) >= .99]
    report.update(initial=initial, stop_sent=stop_sent, final=after,
                  maximum_speed=max((p['speed'] for p in driven), default=0),
                  displacement=math.dist(start, after['position'][:2]),
                  brake_latency=(brake_samples[0]['wall_time'] - stop_sent) if brake_samples else None,
                  stopped=stopped, passed=stopped and bool(driven) and
                  max(p['speed'] for p in driven) <= 1.6)
    print(json.dumps({k: x for k, x in report.items() if k not in ('initial', 'final')}, indent=2), flush=True)
finally:
    try:
        param('/wildos_cmd_vel_to_control', 'enable', False)
        param('/wildos_graph_path_bridge', 'follow_path', False)
    finally:
        b.control.pause()
        # Calibration is before the search. Restore the requested spawn once;
        # no teleport occurs between trash-can search and station return.
        b.vehicles.teleport('ego', (734.294, -11.946, 52.154),
                            (0., 0., .7658087977066911, .6430683364581344), reset=True)
        report['samples'] = samples
        out.joinpath('control_calibration.json').write_text(json.dumps(report, indent=2))
        v.disconnect()
        b.disconnect()
        n.destroy_node()
        rclpy.shutdown()
if not report['passed']:
    raise RuntimeError('Control calibration did not pass')
