"""Audit a CLOSED trial: target progression and independent body-distance scope."""
import bisect
import collections
import json
import math
from pathlib import Path
import sqlite3
import sys

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.arrival_criteria import vehicle_body_distance

trial = Path(sys.argv[1])
result = json.loads((trial / 'result.json').read_text())
assert result['closed']
goal = next(trial.glob('sessions/*/goal_001'))
events = [json.loads(line) for line in (goal / 'events.jsonl').read_text().splitlines()]
query = next(e['elapsed'] for e in events if e.get('kind') == 'query_sent')
first = next(e for e in events if e.get('stamp_ns'))
origin = first['stamp_ns'] - round((first['elapsed'] - query) * 1e9)
duration = min(120., result['search_elapsed_sec'])
events = [e for e in events if 0 <= e['elapsed'] - query <= duration]
db = sqlite3.connect(next((goal / 'interfaces').glob('*.db3')).as_uri() + '?mode=ro', uri=True)
topics = {name: (index, get_message(kind)) for index, name, kind in
          db.execute('select id,name,type from topics')}
index, kind = topics['/spot1/odom']
odoms = [(timestamp, deserialize_message(data, kind)) for timestamp, data in db.execute(
    'select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp',
    (index, origin, origin + round(duration * 1e9)))]
db.close()

def xy(position):
    return [position.x, position.y]

def heading(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

raw = [e for e in events if e.get('status') == 'awaiting_graph_route' and e.get('goal')]
confirmations = [e for e in events if e.get('kind') == 'object_confirmation' and
                 e.get('payload', {}).get('search_confirmed') is True]
target = (confirmations[-1]['payload']['object_position'] if confirmations else
          raw[-1]['goal'] if raw else None)
distances = []
if target:
    for timestamp, odom in odoms:
        pose = odom.pose.pose
        distances.append(dict(query_s=(timestamp-origin)*1e-9, position=xy(pose.position),
                              heading=heading(pose.orientation),
                              body_distance_m=vehicle_body_distance(xy(pose.position), heading(pose.orientation), target, 'ackermann')))
times = [d['query_s'] for d in distances]
milestones = []
for second in range(0, 121, 10):
    if times and times[0] <= second <= times[-1]:
        milestones.append(distances[max(0, bisect.bisect_right(times, second)-1)])
planned = []
for event in events:
    if event.get('status') == 'planned' and event.get('graph_goal') and event.get('actual_start_pose'):
        start = event['actual_start_pose']
        endpoint, requested = event['goal'], event['graph_goal']
        planned.append(dict(query_s=event['elapsed']-query, start=start, endpoint=endpoint,
                            graph_goal=requested,
                            start_target_distance_m=math.dist(start[:2], requested[:2]),
                            endpoint_target_distance_m=math.dist(endpoint[:2], requested[:2])))
report = dict(
    scope='Closed 120s bag and events. Target is the final program-confirmed point, NOT independent semantic ground truth. Direction change is not gearbox telemetry.',
    result=result,
    status_counts=dict(collections.Counter(e['status'] for e in events if e.get('status'))),
    first_raw_goal=raw[0] if raw else None,
    first_confirmation_query_s=confirmations[0]['elapsed']-query if confirmations else None,
    final_program_target=target,
    confirmations=len(confirmations),
    minimum_body_distance=min(distances, key=lambda x: x['body_distance_m']) if distances else None,
    final_pose=distances[-1] if distances else None,
    target_distance_milestones=milestones,
    planned_target_paths=planned,
    program_object_found=[e for e in events if e.get('status') == 'object_found'],
)
(trial / 'target_approach_audit.json').write_text(json.dumps(report, indent=2))
print(json.dumps({key: value for key, value in report.items() if key not in
                  ('result', 'first_raw_goal', 'planned_target_paths')}, indent=2))
