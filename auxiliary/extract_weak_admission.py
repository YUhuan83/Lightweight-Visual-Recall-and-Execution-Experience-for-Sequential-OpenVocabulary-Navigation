"""Closed v40 observations: reject math and real Smac pose counterfactuals."""
import json,math,sqlite3,sys
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.graph_route import forward_arc_arrival_heading
base=Path('/home/ly/execution_continuity_review_20261022');root=base/'v40_observation_heading'
receipt=json.loads((root/'field_result.json').read_text());assert receipt['result']['closed'] and not receipt['postcheck']['processes']
goal=next(Path(receipt['trial']).glob('sessions/*/goal_001'))
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
def rows(name):
    i,c=topics[name]
    return [(t,deserialize_message(b,c)) for t,b in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(i,))]
odom=rows('/spot1/odom');maps=rows('/global_costmap/costmap_raw')
events=[json.loads(l) for l in (goal/'events.jsonl').read_text().splitlines()]
plans=[e for e in events if e.get('kind')=='plan_status']
def ns(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
def h(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def before(seq,t):return next(m for stamp,m in reversed(seq) if stamp<=t)
output=root/'weak_admission_probe';output.mkdir(exist_ok=True);report=[]
for e in plans:
    if e['status'] not in ('visible_target_no_vehicle_path','visible_target_local_fallback'):continue
    stamp=e['stamp_ns'];wanted=e['replay_inputs']['odom_stamp_ns']
    o=next(m for _,m in odom if ns(m)==wanted)
    p=o.pose.pose;start=[p.position.x,p.position.y,h(p.orientation)];target=e['goal']
    bearing=math.atan2(target[1]-start[1],target[0]-start[0]);delta=math.remainder(bearing-start[2],2*math.pi)
    arc=forward_arc_arrival_heading(start[:2],start[2],target,12.)
    radius=math.dist(start[:2],target)/(2*abs(math.sin(delta))) if abs(math.sin(delta))>1e-9 else None
    # Circular endpoint yaw without assuming that the circular path itself
    # is admissible. Smac still decides complete collision/curvature validity.
    geometric_yaw=math.remainder(start[2]+2*delta,2*math.pi)
    goals=[[*target,bearing],[*target,geometric_yaw],[*target,start[2]]]
    raw=before(maps,stamp);i=raw.metadata;file=output/f'case_{len(report)}.txt'
    with file.open('w') as f:
        f.write(' '.join(map(str,[i.size_x,i.size_y,i.resolution,i.origin.position.x,i.origin.position.y,*start,len(goals)]))+'\n')
        for g in goals:f.write(' '.join(map(str,g))+'\n')
        for idx,cost in enumerate(raw.data):
            if cost:f.write(f'{idx} {cost}\n')
    report.append(dict(event=e['status'],stamp_ns=stamp,elapsed=e['elapsed'],start=start,target=target,
        fitted_arc_heading=arc,single_circle_radius_m=radius,forward=abs(delta)<math.pi/2,
        variants=goals,file=str(file),map_stamp_ns=ns(raw)))
db.close()
report=dict(scope='Recorded odom stamp and public raw map. Planner pose counterfactual, not exact private locked map or vehicle acceptance.',rows=report)
(output/'inputs.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
