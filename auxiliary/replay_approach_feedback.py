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
evidence = base/'v45_approach_contract'
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


raw_events=[json.loads(line) for line in (goal/'events.jsonl').read_text().splitlines()]
failed=next(e for e in raw_events if e.get('status')=='confirmed_approach_failed' and e.get('reason')=='no_vehicle_path')
failure_time=failed['stamp_ns']
recorded_monotonic=patch('time.monotonic',lambda:now*1e-9)
recorded_monotonic.start()
def apply_inputs(at):
    n.odom=deepcopy(last(odoms,at));b.odom=deepcopy(n.odom)
    b.odom_received_at=time.monotonic()
    n.occupancy=last(terrain,at)
    n.occupancy_odom=min((m for _,m in odoms),key=lambda m:abs(stamp_ns(m)-stamp_ns(n.occupancy)))
    n.vehicle_grid=b.grid=last(maps,at);n.vehicle_connectivity=None
    b.grid_received_at=time.monotonic();b.local_grid=b.grid;b.local_grid_received_at=time.monotonic()
try:
    for now,data in localizations:
        if now>failure_time:break
        apply_inputs(now)
        n.on_localization(String(data=json.dumps(data)))
    now=failure_time;apply_inputs(now)
    assert n.active_approach is not None and n.confirmed_instance_anchor is not None
    anchor=deepcopy(n.active_approach[1])
    b.active_object_confirmation=dict(anchor,object_position=anchor['object_position'][:2])
    b.expected=b.confirmed_route_goal=tuple(failed['goal'])
    b.confirmed_route_object=tuple(anchor['object_position'][:2])
    b.confirmed_route_radius=float(anchor['radius'])
    b.confirmed_route_started_at=time.monotonic()-10.
    b.deferred_confirmed_approach=json.loads(n.confirmation_pub.publish.call_args.args[0].data)
    before_goal=n.last_goal;before_count=n.confirmation_pub.publish.call_count
    assert b.fail_confirmed_approach('no_vehicle_path',reasons=failed.get('reasons',[]))
    failure_message=b.status_pub.publish.call_args.args[0]
    n.on_navigation_plan_status(failure_message)
    refreshed=n.confirmation_pub.publish.call_count>before_count
    if refreshed:b.on_object_confirmation(n.confirmation_pub.publish.call_args.args[0])
    deferred=deepcopy(b.deferred_confirmed_approach)
    current=(b.odom.pose.pose.position.x,b.odom.pose.pose.position.y)
    traces=[]
    original_blockers=b.pose_blockers
    def trace_blockers(grid,point,heading):
        result=original_blockers(grid,point,heading)
        traces.append(dict(point=list(point),heading=heading,blockers=result))
        return result
    b.pose_blockers=trace_blockers
    started=b.start_live_observation()
    chosen=None
    if started:
        pose=b.waypoint_pub.publish.call_args.args[0].pose
        chosen=[pose.position.x,pose.position.y,yaw(pose.orientation)]
        raw=last(rawmaps,now);m=raw.metadata
        file=probe_dir/'feedback_pose.txt'
        start=[*current,yaw(b.odom.pose.pose.orientation)]
        with file.open('w') as f:
            f.write(' '.join(map(str,[m.size_x,m.size_y,m.resolution,m.origin.position.x,m.origin.position.y,*start,1]))+'\n')
            f.write(' '.join(map(str,chosen))+'\n')
            for index,cost in enumerate(raw.data):
                if cost:f.write(f'{index} {cost}\n')
    report=dict(version=root.name,source_trial=receipt['trial'],failure_stamp_ns=now,
        original_failed_goal=failed['goal'],latest_proposal_before=before_goal,
        proposal_gap_m=math.dist(failed['goal'],before_goal),
        failure_payload=json.loads(failure_message.data),receiver_regenerated=refreshed,
        deferred_after=deferred,recovery_started=started,recovery_pose=chosen,
        recovery_distance_m=math.dist(current,chosen[:2]) if chosen else None,
        rejected_views=list(n.rejected_observation_poses),pose_filter_trace=traces,
        acceptance=False,scope='Actual callbacks; recorded localization/odom/map inputs through first v45 no_vehicle_path. Candidate comparison/action motion history not replayed, no vehicle/action and public map not locked runtime map.')
finally:
    recorded_monotonic.stop();n.destroy_node();b.destroy_node();rclpy.shutdown()
encoded=json.dumps(report,sort_keys=True).encode()
(root/'approach_feedback_replay.json').write_bytes(encoded)
print(json.dumps(report,indent=2))
print('SHA256',hashlib.sha256(encoded).hexdigest())
