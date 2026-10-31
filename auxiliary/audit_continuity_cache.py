"""Read closed field bags: revalidate cached background paths at actual reuse."""
import json
import math
import sqlite3
import sys
from pathlib import Path
from collections import Counter
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.continuous_path import trim_moving_path
from wildos_nav2_adapter.graph_path_bridge import xy, yaw
from wildos_nav2_adapter.graph_route import validate_vehicle_path, validate_local_prefix
from wildos_nav2_adapter.motion_path import motion_sections

root = Path(sys.argv[1])
assert json.loads((root/'result.json').read_text())['closed']
session = next((root/'sessions').glob('session_*'))
bag = next((session/'goal_001/interfaces').glob('*.db3'))
db = sqlite3.connect(f'file:{bag}?mode=ro', uri=True)
topics = {name:(idx,get_message(typ)) for idx,name,typ in db.execute('select id,name,type from topics')}
def rows(topic):
    idx, cls = topics[topic]
    return [(t,deserialize_message(data,cls)) for t,data in db.execute(
        'select timestamp,data from messages where topic_id=? order by timestamp',(idx,))]
plans = [(t,json.loads(m.data)) for t,m in rows('/wildos/plan_status')]
full = rows('/wildos/full_planned_path')
odom = rows('/spot1/odom')
grids = rows('/global_costmap/costmap')
local = rows('/local_costmap/costmap')
routes = rows('/spot1/graphnav_planner/path')
def last(seq,stamp):
    return next((v for t,v in reversed(seq) if t<=stamp),None)
report = dict(root=str(root),rejections=dict(Counter(p.get('extension_rejected') for _,p in plans if p.get('extension_rejected'))),cases=[])
for t,p in plans:
    if p.get('extension_rejected')!='wrong_start':continue
    available = [(pt,path) for pt,path in full if pt<t and t-pt<10e9]
    if not available:continue
    pt,path = available[-1]
    o=last(odom,t);g=last(grids,t);lg=last(local,t);route=last(routes,t)
    pos=xy(o.pose.pose);h=yaw(o.pose.pose)
    trimmed=trim_moving_path(path,pos,h)
    points=[xy(q.pose) for q in path.poses]
    case=dict(reuse_ns=t,path_received_ns=pt,delay_sec=(t-pt)/1e9,
        source=points[0],current=pos,heading=h,offset_m=math.dist(points[0],pos),
        sections=motion_sections(points),poses=len(points),trim_possible=trimmed is not None)
    if trimmed is not None:
        ps=[xy(q.pose) for q in trimmed.poses];hs=[yaw(q.pose) for q in trimmed.poses]
        corridor=[pos]+[xy(q.pose) for q in route.poses]
        case.update(trimmed_poses=len(ps),trimmed_start=ps[0],
            global_issue=validate_vehicle_path(ps,hs,corridor,g,points[-1],start=pos,minimum_turning_radius=8.,heading_quantization_bins=72),
            local_issue=validate_local_prefix(ps,hs,lg,pos,h),
            fixture=dict(path=dict(frame=path.header.frame_id,points=points,headings=[yaw(q.pose) for q in path.poses]),current=[*pos,h]))
    report['cases'].append(case)
output=root.parent.parent/'cache_reuse_audit.json'
output.write_text(json.dumps(report,indent=2))
print(json.dumps({**report,'cases':[{k:v for k,v in c.items() if k!='fixture'} for c in report['cases']]},indent=2))
