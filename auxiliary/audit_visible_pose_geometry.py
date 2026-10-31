"""Compare accepted visual observation poses with the real vehicle geometry."""
import bisect,hashlib,json,math,sqlite3
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.graph_route import forward_arc_arrival_heading
root=Path('/home/ly/execution_continuity_review_20261022/v25_fast_shift')
out=root.parent/'target_contract_20261025'
reports=[]
for trial in sorted((root/'full').glob('trial_*')):
    assert json.loads((trial/'result.json').read_text())['closed']
    goal=next(trial.glob('sessions/*/goal_001'));db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
    topics={n:(i,t) for i,n,t in db.execute('select id,name,type from topics')}
    def load(name):
        tid,typ=topics[name];cls=get_message(typ)
        return [(t,deserialize_message(data,cls)) for t,data in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid,))]
    odom=load('/spot1/odom');by_stamp={m.header.stamp.sec*10**9+m.header.stamp.nanosec:m for _,m in odom}
    hints=[(t,json.loads(m.data)) for t,m in load('/spot1/visible_target_approach')]
    plans={d['stamp_ns']:d for _,m in load('/wildos/plan_status') for d in [json.loads(m.data)]}
    rows=[]
    for stamp,e in sorted(plans.items()):
        if e['status']!='visible_target_approach':continue
        o=by_stamp.get(e['replay_inputs']['odom_stamp_ns'])
        if o is None:rows.append(dict(event=e,error='odom_not_recorded'));continue
        p=o.pose.pose;q=p.orientation;heading=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));xy=(p.position.x,p.position.y)
        h=next((d for t,d in reversed(hints) if t<=stamp and math.dist(d['point'],e['goal'])<.01),None)
        target=e['goal'];angle=math.remainder(math.atan2(target[1]-xy[1],target[0]-xy[0])-heading,2*math.pi);length=math.dist(xy,target)
        arc=forward_arc_arrival_heading(xy,heading,target,12.)
        rows.append(dict(event=e,pose=[*xy,heading],bearing=h['bearing'] if h else None,distance_m=length,body_to_goal_angle_deg=math.degrees(angle),direct_arc_radius_m=length/(2*abs(math.sin(angle))) if abs(math.sin(angle))>1e-8 else None,feasible_arc_heading=arc,required_yaw_change_deg=math.degrees(math.remainder(h['bearing']-heading,2*math.pi)) if h else None))
    reports.append(dict(trial=trial.name,rows=rows));db.close()
encoded=json.dumps(reports,sort_keys=True).encode();(out/'visible_pose_geometry.json').write_bytes(encoded)
print(json.dumps(dict(sha256=hashlib.sha256(encoded).hexdigest(),rounds=reports)))
