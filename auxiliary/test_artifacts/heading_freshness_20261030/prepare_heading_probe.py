"""Actual requested view yaw versus existing vehicle arrival yaw, same start and global map."""
import json,math,sqlite3
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.graph_route import forward_arc_arrival_heading
root=Path('/home/ly/execution_continuity_review_20261022/v71_heading_freshness')
trial=root/'full/trial_20261030_102622'
assert json.loads((trial/'result.json').read_text())['closed']
goal=next(trial.glob('sessions/*/goal_001'))
events=[json.loads(x) for x in (goal/'events.jsonl').read_text().splitlines()]
query=next(x['elapsed'] for x in events if x.get('kind')=='query_sent')
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topic_id,kind=db.execute('select id,type from topics where name=?',('/global_costmap/costmap_raw',)).fetchone()
cls=get_message(kind)
maps={}
for receipt,data in db.execute('select timestamp,data from messages where topic_id=?',(topic_id,)):
    m=deserialize_message(data,cls)
    maps[m.header.stamp.sec*10**9+m.header.stamp.nanosec]=(receipt,m)
db.close();reports=[]
for second in (15.7,40.8,52.2):
    event=next(x for x in events if x.get('status')=='planning' and x['elapsed']-query>=second)
    # Recorded raw costmaps and the bridge's OccupancyGrid have different
    # publication stamps. Use the last raw snapshot received before request;
    # all counterfactual headings use this SAME snapshot.
    receipt,m=max((v for v in maps.values() if v[0]<=event['stamp_ns']),key=lambda v:v[0])
    start=event['bridge_odom_pose'];target=event['requested_goal_pose'];x,y=target[:2]
    goals=[target,[x,y,math.atan2(y-start[1],x-start[0])]]
    arc=forward_arc_arrival_heading(start[:2],start[2],target[:2],12.)
    if arc is not None:goals.append([x,y,arc])
    meta=m.metadata;filename=root/f'heading_probe_{second}.txt'
    with filename.open('w') as f:
        f.write(' '.join(map(str,[meta.size_x,meta.size_y,meta.resolution,meta.origin.position.x,meta.origin.position.y,*start,len(goals)]))+'\n')
        for g in goals:f.write(' '.join(map(str,g))+'\n')
        for i,cost in enumerate(m.data):
            if cost:f.write(f'{i} {cost}\n')
    reports.append(dict(q=event['elapsed']-query,filename=str(filename),start=start,goals=goals,
        variant_order=['recorded_view_yaw','approach_bearing','existing_forward_arc'][:len(goals)],
        map_receipt_ns=receipt,map_age_s=(event['stamp_ns']-receipt)*1e-9,
        scope='Same recorded raw snapshot for all yaw variants; not exact internal planner-map replay',
        bridge_map_header_ns=event['replay_inputs']['global_costmap_stamp_ns']))
(root/'heading_probe_scope.json').write_text(json.dumps(reports,indent=2))
print(json.dumps(reports,indent=2))
