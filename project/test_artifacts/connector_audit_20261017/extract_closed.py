"""Extract read-only published map snapshots; not exact planner private inputs."""
import json, math, sqlite3
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

source=Path('/home/ly/visible_pursuit_repair_20261017')
out=Path('/home/ly/connector_audit_20261017'); out.mkdir(exist_ok=True)
session=source/'field_results/session_20261017_155029'
assert (session/'session_result.json').exists() and (session/'goal_001/result.json').exists()
db=sqlite3.connect((session/'goal_001/interfaces/interfaces_0.db3').as_uri()+'?mode=ro',uri=True)
topics={name:(i,get_message(kind)) for i,name,kind in db.execute('select id,name,type from topics')}
def latest(name,stamp):
    i,kind=topics[name]
    ts,data=db.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',(i,int(stamp))).fetchone()
    return ts,deserialize_message(data,kind)
review=json.loads((source/'execution_loop_review.json').read_text())
cases=[]
for i,path in enumerate(review['full_candidates']):
    ts,grid=latest('/global_costmap/costmap',path['time'])
    header=[grid.info.width,grid.info.height,grid.info.resolution,grid.info.origin.position.x,
            grid.info.origin.position.y,*path['path'][0],*path['path'][-1]]
    (out/f'input{i}.txt').write_text(' '.join(map(str,header))+'\n'+' '.join(map(str,grid.data))+'\n')
    cases.append(dict(index=i,path_stamp=path['time'],map_received_stamp=ts,
        map_source_stamp=grid.header.stamp.sec*10**9+grid.header.stamp.nanosec,
        start=path['path'][0],goal=path['path'][-1]))
changes=[]
for case in review['handoff_cases']:
    hit=case.get('first_center_hit')
    if not hit:continue
    _,before=latest('/global_costmap/costmap',case['full_time'])
    _,after=latest('/global_costmap/costmap',case['event']['stamp_ns'])
    cells=np.asarray(after.data).reshape(after.info.height,after.info.width)
    ys,xs=np.where(cells==100)
    wx=after.info.origin.position.x+(xs+.5)*after.info.resolution
    wy=after.info.origin.position.y+(ys+.5)*after.info.resolution
    ds=np.hypot(wx-hit['xy'][0],wy-hit['xy'][1])
    nearest=[]
    for k in np.argsort(ds)[:5]:
        ix=math.floor((wx[k]-before.info.origin.position.x)/before.info.resolution)
        iy=math.floor((wy[k]-before.info.origin.position.y)/before.info.resolution)
        old=int(before.data[iy*before.info.width+ix]) if 0<=ix<before.info.width and 0<=iy<before.info.height else None
        nearest.append(dict(x=float(wx[k]),y=float(wy[k]),distance=float(ds[k]),old_cost=old,new_cost=100))
    changes.append(dict(stamp=case['event']['stamp_ns'],hit=hit,nearest_lethal=nearest))
result=dict(scope=__doc__,cases=cases,map_changes=changes,topics=list(topics))
(out/'inputs.json').write_text(json.dumps(result,indent=2))
print(json.dumps(dict(cases=len(cases),map_changes=changes,topics=list(topics)),indent=2))
