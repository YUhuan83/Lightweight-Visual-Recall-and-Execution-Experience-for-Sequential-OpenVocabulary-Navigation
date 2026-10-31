"""Read closed 215010 inputs and reconstruct its terminal approach state.

This is callback replay of recorded inputs, not the original private async queue
or a new feasible-path/visual-arrival claim. No action clients are dispatched.
"""
from pathlib import Path
import argparse
import hashlib
import json
import math
import sqlite3
import sys
import time
from types import SimpleNamespace as S
from unittest.mock import Mock

p = argparse.ArgumentParser(); p.add_argument('--source', type=Path, required=True)
p.add_argument('--label', required=True); args = p.parse_args()
sys.path[:0] = [str(args.source/'nav2_adapter'), str(args.source/'official_adapter')]
import rclpy
from rclpy.serialization import deserialize_message
from builtin_interfaces.msg import Time
from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid, Path as NavPath
from geometry_msgs.msg import PoseStamped
from wildos_nav2_adapter.graph_path_bridge import GraphPathBridge

root = Path(__file__).resolve().parent
db = Path('/home/ly/graph_start_attachment_repair_20261020/full/trial_20261020_215010/sessions/session_20261020_215010/goal_001/interfaces/interfaces_0.db3')
connection = sqlite3.connect('file:'+str(db)+'?mode=ro', uri=True)
topics = {name: ident for ident, name in connection.execute('select id,name from topics')}
states = [(t, json.loads(deserialize_message(b, String).data)) for t, b in connection.execute(
    'select timestamp,data from messages where topic_id=? order by timestamp,id', (topics['/wildos/plan_status'],))]
terminal_t, failure = next((t, s) for t, s in states if s.get('status') == 'no_vehicle_path')
held = tuple(failure['goal'])
proposal = [s for t, s in states if t <= terminal_t and s.get('status') == 'confirmed_standoff_update_deferred'
            and s.get('reason') == 'accepted_route_active'][-1]['proposed_goal']
confirmations = [json.loads(deserialize_message(b, String).data) for t, b in connection.execute(
    'select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp,id',
    (topics['/spot1/object_confirmation'], terminal_t))]
seed = [d for d in confirmations if d.get('standoff') and math.dist(d['standoff'], proposal) < 1e-6][-1]
anchor_seed = [d for d in confirmations if d.get('search_confirmed') is True
               and d.get('standoff') and math.dist(d['standoff'], held) <= .6][0]

def last(name, cls):
    row = connection.execute('select data from messages where topic_id=? and timestamp<=? order by timestamp desc,id desc limit 1',
                             (topics[name], terminal_t)).fetchone()
    return deserialize_message(row[0], cls)
odom = last('/spot1/odom', Odometry)
map_name = next(n for n in topics if n.endswith('global_costmap/costmap'))
grid = last(map_name, OccupancyGrid)
connection.close()
def stamp(m): return m.header.stamp.sec*10**9+m.header.stamp.nanosec
now = max(failure['stamp_ns'], stamp(odom), stamp(grid))
report = dict(scope=__doc__, source_root=str(args.source), terminal_bag_ns=terminal_t,
              source_db=str(db), held_goal=held, proposed_goal=proposal, source_failure=failure,
              anchor_seed=anchor_seed, retained_seed=seed, complete_navigation_passed=False,
              synthetic_state_reconstruction=True, actual_recorded_messages=True)
rclpy.init(); n = GraphPathBridge()
try:
    n.get_clock = lambda: S(now=lambda: S(nanoseconds=now,
        to_msg=lambda: Time(sec=now//10**9, nanosec=now%10**9)))
    n.query = failure['query']; n.query_generation = failure['query_generation']; n.query_started_ns = 1
    n.carrier_model = 'ackermann'; n.minimum_turning_radius = 8.; n.planner_id = 'CarLike'
    n.on_odom(odom); n.on_map(grid)
    assert n.odom is not None and n.grid is not None and n.input_freshness_issue() is None
    n.expected = n.confirmed_route_goal = held
    n.confirmed_route_object = tuple(anchor_seed['object_position'][:2])
    n.confirmed_route_radius = float(anchor_seed['radius']); n.confirmed_route_started_at = time.monotonic()-20
    n.active_object_confirmation = dict(seed)
    n.deferred_confirmed_approach = dict(seed)
    path = NavPath(); path.header.frame_id = 'spot1/odom'; path.header.stamp = n.get_clock().now().to_msg()
    for x, y in failure['graph_route']:
        point = PoseStamped(); point.header = path.header; point.pose.position.x = x; point.pose.position.y = y
        point.pose.orientation.w = 1.; path.poses.append(point)
    n.route = n.latest_route = path; n.route_endpoint = held
    n.candidates = [(*g, 0.) for g in failure['candidate_goals']]
    n.candidate_index = len(n.candidates); n.failure_reasons = list(failure['reasons'])
    n.waypoint_pub = Mock(); n.instance_release_pub = Mock()
    n.try_candidate()
    report['immediate_status'] = n.last_status
    n.monitor_observation()
    report.update(final_status=n.last_status, expected=n.expected,
        release_pending=n.instance_release_pending is not None,
        object_found=n.object_found,
        navigation_handoff=n.expected != held or n.instance_release_pending is not None,
        commands_sent=0)
    report['source_identity'] = {}
    for package, name in [('wildos_nav2_adapter', 'confirmed_approach'), ('wildos_nav2_adapter', 'target_observation'),
                          ('wildos_nav2_adapter', 'graph_path_bridge')]:
        module = __import__(package+'.'+name, fromlist=[name]); source = Path(module.__file__).resolve()
        report['source_identity'][name] = dict(path=str(source), sha256=hashlib.sha256(source.read_bytes()).hexdigest())
finally:
    n.destroy_node(); rclpy.shutdown()
(root/(args.label+'.json')).write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps({k: report[k] for k in ('held_goal','proposed_goal','expected','navigation_handoff','release_pending','final_status')}, indent=2))
