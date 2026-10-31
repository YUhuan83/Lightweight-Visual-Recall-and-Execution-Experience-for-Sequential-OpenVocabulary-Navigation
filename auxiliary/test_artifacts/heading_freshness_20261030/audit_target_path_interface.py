"""Read exact received GraphNav paths associated with accepted vehicle plans; CLOSED bag only."""
import json,sqlite3,sys,math
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
trial=Path(sys.argv[1]);assert json.loads((trial/'result.json').read_text())['closed']
goal=next(trial.glob('sessions/*/goal_001'))
events=[json.loads(x) for x in (goal/'events.jsonl').read_text().splitlines()]
query=next(e['elapsed'] for e in events if e.get('kind')=='query_sent')
first=next(e for e in events if e.get('stamp_ns'))
origin=first['stamp_ns']-round((first['elapsed']-query)*1e9)
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
def messages(name):
    i,c=topics[name]
    return [(t,deserialize_message(b,c)) for t,b in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(i,))]
def ns(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
paths={ns(m):m for _,m in messages('/spot1/graphnav_planner/path')}
statuses={d['stamp_ns']:d for _,m in messages('/spot1/graphnav_planner/route_status') for d in [json.loads(m.data)]}
rows=[]
for e in events:
    q=e['elapsed']-query
    if e.get('status')!='planned' or not 65<=q<=120:continue
    stamp=e['replay_inputs']['route_stamp_ns'];path=paths.get(stamp)
    points=[[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in path.poses] if path else []
    rows.append(dict(q=q,route_stamp_ns=stamp,vehicle_start=e.get('actual_start_pose'),
        vehicle_endpoint=e.get('goal'),requested_object_xy=e.get('graph_goal'),
        graph_status=statuses.get(stamp),graph_points=points))
raw=[(t,m) for t,m in messages('/spot1/raw_object_waypoint') if origin<=t<=origin+120000000000]
odom=[(t,m) for t,m in messages('/spot1/odom') if origin<=t<=origin+120000000000]
report=dict(scope='Exact received path-stamp association, not proof of optimality or collision truth; final raw target is not semantic ground truth',plans=rows)
if raw and odom:
    p=raw[-1][1].pose.position;target=[p.x,p.y,p.z]
    p=odom[-1][1].pose.pose.position;position=[p.x,p.y,p.z]
    report.update(first_raw_q=(raw[0][0]-origin)*1e-9,last_raw_q=(raw[-1][0]-origin)*1e-9,
        final_raw_target=target,final_odom=position,final_xyz_to_last_raw_m=math.dist(target,position))
db.close();(trial/'target_path_interface_audit.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
