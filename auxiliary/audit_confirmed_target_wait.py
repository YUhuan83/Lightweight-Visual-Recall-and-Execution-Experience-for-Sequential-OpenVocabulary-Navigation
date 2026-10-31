"""Read-only closed-bag audit: distinguish final arrival and failed approach."""
import collections
import json
import math
import sqlite3
import sys
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.arrival_criteria import vehicle_body_distance
from wildos_nav2_adapter.graph_path_bridge import xy,yaw

trial=Path(sys.argv[1])
assert json.loads((trial/'result.json').read_text())['closed']
goal=next(trial.glob('sessions/*/goal_001'))
assert (goal/'stop.json').exists()
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
types={name:(idx,get_message(typ)) for idx,name,typ in db.execute('select id,name,type from topics')}
def rows(name):
    idx,cls=types[name]
    return [(stamp,deserialize_message(blob,cls)) for stamp,blob in db.execute(
        'select timestamp,data from messages where topic_id=? order by timestamp',(idx,))]
odom=rows('/spot1/odom')
confirmations=[(t,json.loads(m.data)) for t,m in rows('/spot1/object_confirmation')]
valid=[(t,d) for t,d in confirmations if d.get('search_confirmed') is True]
report=dict(trial=str(trial),messages=len(confirmations),search_confirmed=len(valid),
    ready=sum(d.get('approach_goal_ready') is True for _,d in valid),
    failures=dict(collections.Counter(d.get('approach_failure_reason') for _,d in valid)),examples=[])
for t,d in valid:
    o=next(o for stamp,o in reversed(odom) if stamp<=t)
    p=xy(o.pose.pose);h=yaw(o.pose.pose)
    case={k:d.get(k) for k in ('observation_stamp_ns','object_position','observer_position','source','radius','inliers','observations','approach_failure_reason','approach_goal_ready','standoff')}
    case.update(received_ns=t,body_distance_m=vehicle_body_distance(p,h,d['object_position'],'ackermann'),vehicle=[*p,h])
    report['examples'].append(case)
db.close()
output=trial.parents[1]/('confirmation_wait_'+trial.name+'.json')
output.write_text(json.dumps(report,indent=2))
print(json.dumps({**report,'examples':report['examples'][:3]+report['examples'][-2:]},indent=2))
