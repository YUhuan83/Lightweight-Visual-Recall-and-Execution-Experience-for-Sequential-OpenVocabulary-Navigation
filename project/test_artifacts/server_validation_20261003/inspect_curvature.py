"""Check tracking and measured motion before saturation-related cancellations."""
from pathlib import Path
import json
import sqlite3
import sys
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.graph_route import distance_to_route, distance
from wildos_nav2_adapter.graph_path_bridge import xy

ROOT=Path('/home/ly/town_memory_validation_20261003')
trial=ROOT/sys.argv[1]
db=sqlite3.connect(f"file:{next((trial/'rosbag').glob('*.db3'))}?mode=ro",uri=True)
topics={name:(identifier,get_message(kind)) for identifier,name,kind in db.execute('SELECT id,name,type FROM topics')}
def read(name):
    identifier,kind=topics[name]
    return [(ts,deserialize_message(blob,kind)) for ts,blob in db.execute(
        'SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',(identifier,))]
paths=[(t,[xy(p.pose) for p in m.poses]) for t,m in read('/wildos/planned_path')]
odoms=[(t,xy(m.pose.pose)) for t,m in read('/spot1/odom')]
controls=[(t,json.loads(m.data)) for t,m in read('/wildos/control_status')]
result=[]
for t,m in read('/wildos/plan_status'):
    d=json.loads(m.data)
    if d.get('reason')!='controller_curvature_infeasible':
        continue
    route=next((p for tp,p in reversed(paths) if tp<=t),None)
    recent=[(to,p) for to,p in odoms if t-2.5e9<to<=t]
    if not route or len(recent)<2:
        continue
    cmd=[c for tc,c in controls if t-2.5e9<tc<=t and c.get('actuating')]
    result.append({'cancel_stamp_ns':t,'graph_goal':d.get('graph_goal'),
        'observed_distance_m':sum(distance(a,b) for (_,a),(_,b) in zip(recent,recent[1:])),
        'max_tracking_error_m':max(distance_to_route(p,route) for _,p in recent),
        'raw_curvature_range':[min(c['raw_curvature'] for c in cmd),max(c['raw_curvature'] for c in cmd)] if cmd else None,
        'applied_curvature_range':[min(c['applied_curvature'] for c in cmd),max(c['applied_curvature'] for c in cmd)] if cmd else None})
(trial/'curvature_diagnosis.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
