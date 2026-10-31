"""Closed v33: exact requested poses and map-clear straight controls."""
import json,math,sqlite3
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.graph_path_bridge import xy,yaw
from wildos_nav2_adapter.graph_route import forward_arc_arrival_heading,footprint_blockers
from beamng_wildos_adapter.mapping import safe_travel_distance
root=Path('/home/ly/execution_continuity_review_20261022/v33_integrated_progress')
receipt=json.loads((root/'field_result.json').read_text());assert receipt['result']['closed'] and not receipt['postcheck']['processes']
goal=next(Path(receipt['trial']).glob('sessions/*/goal_001'))
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={name:(idx,get_message(typ)) for idx,name,typ in db.execute('select id,name,type from topics')}
def rows(name):
    idx,cls=topics[name]
    return [(t,deserialize_message(blob,cls)) for t,blob in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(idx,))]
plans=[(t,json.loads(m.data)) for t,m in rows('/wildos/plan_status')]
odom=rows('/spot1/odom');maps=rows('/global_costmap/costmap');locals=rows('/local_costmap/costmap');terrain=rows('/spot1/local_map');rawmaps=rows('/global_costmap/costmap_raw')
def last(seq,t):return next((m for stamp,m in reversed(seq) if stamp<=t),None)
out=root/'observation_pose_probe';out.mkdir(exist_ok=True)
report=[]
for t,p in plans:
    if (p.get('status')!='planning' or p.get('source')!='visible_target_via_graphnav' or p.get('background') or p.get('candidate_index')!=0):continue
    start=p['bridge_odom_pose'];old=p['requested_goal_pose']
    if not 2<=math.dist(start[:2],old[:2])<=10:continue
    if any(math.dist(start[:2],r['start'][:2])<2 for r in report):continue
    global_map=last(maps,t);local_map=last(locals,t);ground=last(terrain,t);raw=last(rawmaps,t)
    go=next(m for _,m in odom if m.header.stamp==ground.header.stamp)
    map_pose=go.pose.pose;map_yaw=yaw(map_pose)
    grid=np.asarray(ground.data).reshape(ground.info.height,ground.info.width)
    straight=None
    for travel in np.arange(8.,1.99,-.25):
        endpoint=[start[0]+travel*math.cos(start[2]),start[1]+travel*math.sin(start[2]),start[2]]
        # This control only investigates map-clear progress in front of the body.
        target_local=[travel*math.cos(start[2]-map_yaw),travel*math.sin(start[2]-map_yaw)]
        safe=safe_travel_distance(grid,ground.info.resolution,(ground.info.origin.position.x,ground.info.origin.position.y),target_local,travel,2.,max(ground.info.resolution,.5))
        if safe+.1<travel:continue
        if any(footprint_blockers(g,(start[0]+d*math.cos(start[2]),start[1]+d*math.sin(start[2])),start[2],limit=1) for d in np.arange(0.,travel+.01,.25) for g in (global_map,local_map)):continue
        straight=endpoint;break
    arc=forward_arc_arrival_heading(start[:2],start[2],old[:2],12.)
    goals=[old]
    labels=['recorded_pose']
    if arc is not None:goals.append([old[0],old[1],arc]);labels.append('same_point_arc_heading')
    if straight is not None:goals.append(straight);labels.append('map_clear_straight_control')
    i=raw.metadata;file=out/f'case_{len(report)}.txt'
    with file.open('w') as f:
        f.write(' '.join(map(str,[i.size_x,i.size_y,i.resolution,i.origin.position.x,i.origin.position.y,*start,len(goals)]))+'\n')
        for g in goals:f.write(' '.join(map(str,g))+'\n')
        for idx,cost in enumerate(raw.data):
            if cost:f.write(f'{idx} {cost}\n')
    report.append(dict(file=str(file),stamp_ns=t,request_serial=p.get('request_serial'),start=start,goals=goals,labels=labels,arc_feasible=arc is not None))
    if len(report)>=4:break
db.close();(out/'manifest.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
