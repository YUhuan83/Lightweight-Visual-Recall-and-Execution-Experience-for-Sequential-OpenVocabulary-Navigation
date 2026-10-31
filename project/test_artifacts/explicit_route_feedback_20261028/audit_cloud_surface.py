"""Trace measured obstacle points into the v63 transverse costmap stripes."""
import json,math,sqlite3
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.pointcloud import decode_xyz,local_ground_plane
from wildos_nav2_adapter.math3d import matrix_from_quaternion
from wildos_nav2_adapter.vehicle_geometry import SBR_BASE_OFFSET
root=Path('/home/ly/execution_continuity_review_20261022/v63_explicit_route_feedback')
trial=root/'full/trial_20261028_230923';assert json.loads((trial/'result.json').read_text())['closed']
goal=next(trial.glob('sessions/*/goal_001'));out=root/'request_audit';out.mkdir(exist_ok=True)
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
print('cloud topics',[n for n in topics if 'points' in n or 'lidar' in n])
def stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
def last(name,t):
    i,kind=topics[name];ns,b=db.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',(i,t)).fetchone()
    return ns,deserialize_message(b,kind)
i,kind=topics['/spot1/odom']
odoms={stamp(o):o for (b,) in db.execute('select data from messages where topic_id=?',(i,)) for o in [deserialize_message(b,kind)]}
summary=[]
events=[json.loads(l) for l in (goal/'events.jsonl').read_text().splitlines()]
for serial in (24,34):
    e=next(e for e in events if e.get('status')=='planning' and e['request_serial']==serial)
    rows=[]
    _,mark_message=last('/wildos/points',e['stamp_ns'])
    for name in ('/wildos/points','/wildos/clearing_points','/spot1/ouster/front/points_filtered'):
        if name not in topics:continue
        rt,m=last(name,e['stamp_ns']);p=decode_xyz(m);o=odoms.get(stamp(m))
        if name=='/spot1/ouster/front/points_filtered' and stamp(m)!=stamp(mark_message):
            i,kind=topics[name]
            matches=[(t,decoded) for t,b in db.execute('select timestamp,data from messages where topic_id=? and timestamp>=? and timestamp<=?',(i,e['stamp_ns']-2_000_000_000,e['stamp_ns'])) for decoded in [deserialize_message(b,kind)] if stamp(decoded)==stamp(mark_message)]
            assert len(matches)==1, (serial,len(matches))
            rt,m=matches[0];p=decode_xyz(m);o=odoms.get(stamp(m))
        if o is None:
            rows.append(dict(topic=name,error='no exact source odom'));continue
        q=o.pose.pose.orientation;t=o.pose.pose.position
        r=np.array(matrix_from_quaternion([q.x,q.y,q.z,q.w]));base=np.array([t.x,t.y,t.z])
        if m.header.frame_id=='spot1/ouster/front/os_lidar':p=p-np.array(SBR_BASE_OFFSET);world=p@r.T+base
        elif m.header.frame_id=='spot1/base_link':world=p@r.T+base
        elif m.header.frame_id=='spot1/odom':world=p
        else:
            rows.append(dict(topic=name,error='unhandled frame '+m.header.frame_id));continue
        region=(world[:,0]>728)&(world[:,0]<740)&(world[:,1]>-82)&(world[:,1]<-71)
        np.savez_compressed(out/f'request_{serial}_{name.replace("/","_")}.npz',body=p,world=world,source_stamp_ns=stamp(m))
        rows.append(dict(topic=name,receipt_age_s=(e['stamp_ns']-rt)*1e-9,source_stamp_ns=stamp(m),frame=m.header.frame_id,points=len(p),region_points=int(region.sum()),region_world_z_quantiles=np.quantile(world[region,2],[0,.1,.5,.9,1]).tolist() if np.any(region) else []))
    summary.append(dict(request=serial,clouds=rows))
db.close();(out/'cloud_scope.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
