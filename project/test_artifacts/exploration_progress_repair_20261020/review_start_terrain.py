"""Closed-field terrain at the isolated graph start versus actual odometry position."""
from pathlib import Path
import hashlib
import json
import math
import sqlite3
import numpy as np
from rclpy.serialization import deserialize_message,serialize_message
from rosidl_runtime_py.utilities import get_message
from beamng_wildos_adapter.geometry import quaternion_matrix

ROOT=Path(__file__).resolve().parent
trial=ROOT/'trial_20261020_180816';session=trial/'sessions/session_20261020_180816'
assert json.loads((trial/'result.json').read_text())['closed']
analysis=json.loads((ROOT/'graph_analysis_final.json').read_text())['snapshots']
bad=[x for x in analysis if x['physical_reachable']==1]
db=session/'goal_001/interfaces/interfaces_0.db3'
c=sqlite3.connect('file:'+str(db)+'?mode=ro',uri=True)
topics={n:(i,get_message(t)) for i,n,t in c.execute('SELECT id,name,type FROM topics')}
out=ROOT/'terrain_start_review';out.mkdir(exist_ok=False)
def ns(t):return t.sec*10**9+t.nanosec
def around(topic,stamp):
    i,cls=topics[topic]
    return [deserialize_message(data,cls) for data, in c.execute('SELECT data FROM messages WHERE topic_id=? AND timestamp BETWEEN ? AND ? ORDER BY timestamp,id',(i,stamp-3_000_000_000,stamp+3_000_000_000))]
results=[]
for seq,snapshot in enumerate((bad[0],bad[-1])):
    stamp=ns(type('T',(),snapshot['current']['robot_traversability'][0]['observed_at'])())
    terrain=min(around('/spot1/local_terrain',stamp),key=lambda m:abs(ns(m.header.stamp)-stamp))
    assert ns(terrain.header.stamp)==stamp,(ns(terrain.header.stamp),stamp)
    assert terrain.outer_start_index==terrain.inner_start_index==0
    transforms=[t for m in around('/tf',stamp) for t in m.transforms if t.header.frame_id=='spot1/odom' and t.child_frame_id==terrain.header.frame_id]
    tf=min(transforms,key=lambda t:abs(ns(t.header.stamp)-stamp));assert ns(tf.header.stamp)==stamp
    q=tf.transform.rotation;rot=quaternion_matrix([q.x,q.y,q.z,q.w]);t=tf.transform.translation;origin=np.array([t.x,t.y,t.z])
    n=round(terrain.info.length_x/terrain.info.resolution);res=terrain.info.resolution
    assert n==round(terrain.info.length_y/res)
    layers={name:np.asarray(m.data).reshape(n,n)[::-1,::-1] for name,m in zip(terrain.layers,terrain.data)}
    yy,xx=np.indices((n,n));x=(xx+.5-n/2)*res+terrain.info.pose.position.x;y=(yy+.5-n/2)*res+terrain.info.pose.position.y
    occupied=layers['traversability']>.99
    obs=np.column_stack((x[occupied],y[occupied],np.zeros(np.count_nonzero(occupied))))
    observations=[]
    for label,point in (('graph_current',snapshot['current_xy']),('actual_odom',snapshot['odom_xy'])):
        world=np.array([point[0],point[1],origin[2]]);local=(world-origin)@rot
        ix=math.floor((local[0]-terrain.info.pose.position.x+n*res/2)/res);iy=math.floor((local[1]-terrain.info.pose.position.y+n*res/2)/res)
        distances=np.linalg.norm(obs[:,:2]-local[:2],axis=1);order=np.argsort(distances)[:8]
        nearest=[]
        for k in order:
            ol=obs[k];ow=ol@rot.T+origin
            j=math.floor((ol[0]-terrain.info.pose.position.x+n*res/2)/res);i=math.floor((ol[1]-terrain.info.pose.position.y+n*res/2)/res)
            nearest.append(dict(distance=float(distances[k]),world_xy=ow[:2].tolist(),local_xy=ol[:2].tolist(),layers={name:float(a[i,j]) for name,a in layers.items()}))
        observations.append(dict(label=label,world_xy=point,local_xy=local[:2].tolist(),index=[iy,ix],
                                 cell={name:float(a[iy,ix]) for name,a in layers.items()},nearest_occupied=nearest))
    for name,msg in (('terrain',terrain),('tf',tf)):
        data=serialize_message(msg);(out/('%02d_%s.cdr'%(seq,name))).write_bytes(data)
    results.append(dict(source_ns=snapshot['source_ns'],physical_observed_ns=stamp,terrain_ns=ns(terrain.header.stamp),
                        transform_ns=ns(tf.header.stamp),frame=terrain.header.frame_id,resolution=res,observations=observations))
c.close()
report=dict(scope=__doc__,results=results,limits='Nearest occupied cell centres; not full vehicle footprint proof or independently verified world obstacle classification.',complete_navigation_passed=False)
(out/'result.json').write_text(json.dumps(report,indent=2,allow_nan=True)+'\n')
print(json.dumps(report,indent=2,allow_nan=True))
