"""Counterfactual CLOSED v43 target handoff, real callbacks and recorded maps.

Release/clear service replies are explicitly controlled. No vehicle/actions are
simulated and historical vehicle poses are retained after the counterfactual.
"""
import bisect
from concurrent.futures import Future
from copy import deepcopy
import hashlib
import inspect
import json
import math
from pathlib import Path
import sqlite3
import sys
import time
from types import SimpleNamespace as S
from unittest.mock import Mock, patch

import rclpy
from builtin_interfaces.msg import Time
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String
from beamng_wildos_adapter.goal_standoff import GoalStandoff
from wildos_nav2_adapter.continuous_graph_path_bridge import ContinuousGraphPathBridge

base = Path('/home/ly/execution_continuity_review_20261022')
root = base/sys.argv[1]
evidence = base/'v43_abort_recovery'
receipt = json.loads((evidence/'field_result.json').read_text())
assert receipt['result']['closed'] and not receipt['postcheck']['processes']
goal = next(Path(receipt['trial']).glob('sessions/*/goal_001'))
db = sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro', uri=True)
topics = {n: (i, get_message(t)) for i, n, t in db.execute('select id,name,type from topics')}


def rows(name):
    i, cls = topics[name]
    return [(t, deserialize_message(blob, cls)) for t, blob in
        db.execute('select timestamp,data from messages where topic_id=? order by timestamp', (i,))]


odoms = rows('/spot1/odom')
terrain = rows('/spot1/local_map')
maps = rows('/global_costmap/costmap')
rawmaps = rows('/global_costmap/costmap_raw')
localizations = [(t, json.loads(m.data)) for t, m in rows('/spot1/object_localization')]
queries = rows('/spot1/object_search/query')
query_time = next(t for t, m in queries if m.data == 'green residential house')
db.close()
odom_times = [t for t, _ in odoms]


def last(seq, stamp):
    return next((m for t, m in reversed(seq) if t <= stamp), None)


def stamp_ns(msg):
    return msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


rclpy.init()
n, b = GoalStandoff(), ContinuousGraphPathBridge()
now = query_time
clock = lambda: S(now=lambda: S(nanoseconds=now,
    to_msg=lambda: Time(sec=now//10**9, nanosec=now%10**9)))
for node in (n, b):
    node.get_clock = clock
original_parameter = n.get_parameter
n.get_parameter = lambda name: S(value=3.) if name == 'standoff' else original_parameter(name)
n.confirmation_window_sec = 60.
n.observation_minimum_turning_radius = 12.
n.active_query = b.query = 'green residential house'
n.query_started_ns = b.query_started_ns = query_time
b.query_reset_pending = False
b.carrier_model = 'ackermann'
b.minimum_turning_radius = 8.
b.planner_id = 'CarLike'
b.full_path_execution = True
bridge_parameter = b.get_parameter
b.get_parameter = lambda name: S(value=12.) if name == 'planner_minimum_turning_radius' else bridge_parameter(name)
for node in (n, b):
    node.get_logger = Mock()

b.waypoint_pub = Mock()
b.status_pub = Mock()
b.instance_release_pub = Mock()
b.graph_reset_client = Mock()
b.graph_search_reset_client = b.graph_reset_client
b.graph_reset_client.service_is_ready.return_value = False
n.publisher = Mock()
n.confirmation_pub = Mock()
n.instance_ack_pub = Mock()
published = 0
releases = []
events = []
cases = []
probe_dir = root/'instance_route_probe'
probe_dir.mkdir(exist_ok=True)

recorded_monotonic=patch('time.monotonic',lambda: now*1e-9)
recorded_monotonic.start()
try:
    for now, data in localizations:
        odom = last(odoms, now)
        n.odom = deepcopy(odom)
        b.odom = deepcopy(odom)
        b.odom_received_at = time.monotonic()
        n.occupancy = last(terrain, now)
        n.occupancy_odom = min((m for _, m in odoms),
            key=lambda m: abs(stamp_ns(m)-stamp_ns(n.occupancy)))
        assert abs(stamp_ns(n.occupancy_odom)-stamp_ns(n.occupancy)) <= 200_000_000
        n.vehicle_grid = b.grid = last(maps, now)
        n.vehicle_connectivity = None
        b.grid_received_at = time.monotonic()
        b.local_grid = b.grid
        b.local_grid_received_at = time.monotonic()
        n.on_localization(String(data=json.dumps(data)))
        calls = n.confirmation_pub.publish.call_args_list
        for call in calls[published:]:
            msg = call.args[0]
            confirmation = json.loads(msg.data)
            b.on_object_confirmation(msg)
            events.append(dict(received_ns=now, source_stamp_ns=data['observation_stamp_ns'],
                confirmation=confirmation, consumer_expected=b.expected,
                object_found=b.object_found))
            if b.instance_release_pending is not None:
                request = deepcopy(b.instance_release_pending)
                n.on_instance_release(String(data=json.dumps(request)))
                ack = n.instance_ack_pub.publish.call_args.args[0]
                b.on_instance_release_ack(ack)
                # Controlled successful GraphNav release reply; no old Action
                # was created by this component-only counterfactual replay.
                future = Future()
                future.set_result(S(success=True))
                b.on_graph_reset_done(future, b.query_generation,
                                     b.confirmation_not_before_ns)
                releases.append(dict(received_ns=now, request=request,
                    producer_anchor_cleared=n.confirmed_instance_anchor is None,
                    consumer_release_pending=b.instance_release_pending,
                    graph_clear_pending=b.query_reset_pending))
            if (releases and confirmation.get('search_confirmed') is True
                    and confirmation.get('standoff') is not None and len(cases) < 4):
                point = confirmation['standoff']
                heading = confirmation.get('standoff_navigation_heading')
                if heading is None:
                    target = confirmation['object_position']
                    heading = math.atan2(target[1]-point[1], target[0]-point[0])
                pose = odom.pose.pose
                start = [pose.position.x, pose.position.y, yaw(pose.orientation)]
                raw = last(rawmaps, now)
                metadata = raw.metadata
                out = probe_dir/f'case_{len(cases)}.txt'
                with out.open('w') as f:
                    f.write(' '.join(map(str, [metadata.size_x, metadata.size_y,
                        metadata.resolution, metadata.origin.position.x,
                        metadata.origin.position.y, *start, 1]))+'\n')
                    f.write(' '.join(map(str, [*point, heading]))+'\n')
                    for index, cost in enumerate(raw.data):
                        if cost:
                            f.write(f'{index} {cost}\n')
                cases.append(dict(file=str(out), received_ns=now, start=start,
                    goal=[*point, heading], object_position=confirmation['object_position'],
                    instance_id=confirmation['instance_id'],
                    observation_stamp_ns=confirmation['observation_stamp_ns'],
                    raw_map_age_s=(now-stamp_ns(raw))*1e-9))
        published = len(calls)
finally:
    recorded_monotonic.stop()
    n.destroy_node()
    b.destroy_node()
    rclpy.shutdown()

report = dict(version=root.name, source_sha256={str(inspect.getfile(cls)):hashlib.sha256(Path(inspect.getfile(cls)).read_bytes()).hexdigest() for cls in (GoalStandoff,ContinuousGraphPathBridge)}, source_trial=receipt['trial'], query_received_ns=query_time,
    localization_count=len(localizations), release_count=len(releases), releases=releases,
    confirmations_after_release=sum(e['confirmation'].get('search_confirmed') is True
        and bool(releases) and e['received_ns'] > releases[0]['received_ns'] for e in events),
    cases=cases, events=events, acceptance=False,
    scope='Real producer/consumer callbacks and recorded maps/poses, monotonic deltas driven by recorded receipt time. Controlled successful release ACK and GraphNav reply; no executing Action or vehicle and no semantic ground truth assertion.')
encoded = json.dumps(report, sort_keys=True).encode()
(root/'instance_route_replay.json').write_bytes(encoded)
print(json.dumps(dict(version=root.name, release_count=len(releases),
    first_release_query_s=((releases[0]['received_ns']-query_time)*1e-9 if releases else None),
    confirmations_after_release=report['confirmations_after_release'], cases=cases,
    sha256=hashlib.sha256(encoded).hexdigest(), scope=report['scope']), indent=2))
