"""Closed-bag evidence only: locate lethal cells intersecting forward footprints."""
from pathlib import Path
import json, math, sqlite3
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

root = Path('/home/ly/execution_continuity_review_20261022')
trial = root/'v8/full/trial_20261023_085412'
assert json.loads((trial/'result.json').read_text())['closed']
goal = next(trial.glob('sessions/*/goal_001'))
db = sqlite3.connect(next(goal.rglob('*.db3')).as_uri()+'?mode=ro', uri=True)
topics = {n:(i,t) for i,n,t in db.execute('select id,name,type from topics')}
events = [json.loads(s) for s in (goal/'events.jsonl').read_text().splitlines()]
p = next(e for e in events if e.get('kind')=='plan_status')
origin = p['stamp_ns']/1e9-p['elapsed']
stamp = int((origin+18.087)*1e9)

def latest(name):
    i,t = topics[name]
    row=db.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',(i,stamp)).fetchone()
    return (row[0],deserialize_message(row[1],get_message(t))) if row else (None,None)

_, grid=latest('/global_costmap/costmap')
terrain_ts,terrain=latest('/spot1/local_terrain')
terrain_stamp=terrain.header.stamp.sec*10**9+terrain.header.stamp.nanosec
transforms={}
tid,ttype=topics['/tf']
for _,raw in db.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp',(tid,stamp)):
    for transform in deserialize_message(raw,get_message(ttype)).transforms:
        ns=transform.header.stamp.sec*10**9+transform.header.stamp.nanosec
        if ns<=terrain_stamp: transforms[transform.child_frame_id]=transform
terrain_tf=transforms[terrain.header.frame_id]
assert terrain_tf.header.frame_id==grid.header.frame_id
tq=terrain_tf.transform.rotation
angle=math.atan2(2*(tq.w*tq.z+tq.x*tq.y),1-2*(tq.y*tq.y+tq.z*tq.z))
tc,ts=math.cos(angle),math.sin(angle)
translation=terrain_tf.transform.translation
layers={name:np.array(values.data).reshape(80,80)[::-1,::-1] for name,values in zip(terrain.layers,terrain.data)}
w,h=grid.info.width,grid.info.height
res=grid.info.resolution
ox,oy=grid.info.origin.position.x,grid.info.origin.position.y
lethal=[(i%w,i//w) for i,c in enumerate(grid.data) if c==100]

def inside(x,y,poly):
    yes=False
    for a,b in zip(poly,poly[1:]+poly[:1]):
        if (a[1]>y)!=(b[1]>y) and x<(b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0]:
            yes=not yes
    return yes

clouds={}
for name in topics:
    if 'PointCloud2' not in topics[name][1]: continue
    ts,msg=latest(name)
    if msg is None: continue
    offsets={f.name:f.offset for f in msg.fields}
    arr=np.column_stack([np.ndarray((msg.height,msg.width),dtype=('>' if msg.is_bigendian else '<')+'f4',buffer=bytes(msg.data),offset=offsets[a],strides=(msg.row_step,msg.point_step)).reshape(-1) for a in ('x','y','z')])
    arr=arr[np.isfinite(arr).all(axis=1)]
    clouds[name]=(arr,dict(frame=msg.header.frame_id,bag_age=(stamp-ts)/1e9,fields=list(offsets),points=len(arr)))

report={'scope':'lethal cell centers inside padded footprint; boundary-only cells not included; public quantized map', 'topics':topics,'clouds':{n:v[1] for n,v in clouds.items()},'curves':[]}
for k in range(3):
    poses=[list(map(float,s.split())) for s in (root/f'incoming/early_shunts_probe.txt.forward_{k}').read_text().splitlines()]
    hits={}
    for index,(x,y,yaw) in enumerate(poses):
        c,s=math.cos(yaw),math.sin(yaw)
        poly=[(x+c*a-s*b,y+s*a+c*b) for a,b in [(3.41,1.06),(3.41,-1.06),(-1.06,-1.06),(-1.06,1.06)]]
        for mx,my in lethal:
            wx,wy=ox+(mx+.5)*res,oy+(my+.5)*res
            if inside(wx,wy,poly): hits.setdefault((mx,my),[]).append(index)
    cells=[]
    for (mx,my),indices in sorted(hits.items()):
        wx,wy=ox+(mx+.5)*res,oy+(my+.5)*res
        cell=dict(cell=[mx,my],world=[wx,wy],first_pose=indices[0],last_pose=indices[-1],cloud_nearby={})
        dx,dy=wx-translation.x,wy-translation.y
        lx,ly=tc*dx+ts*dy,-ts*dx+tc*dy
        tx,ty=int(math.floor((lx+20)/.5)),int(math.floor((ly+20)/.5))
        cell['terrain_local_xy']=[lx,ly]
        cell['terrain_bag_age']=(stamp-terrain_ts)/1e9
        cell['terrain_tf_age']=(terrain_stamp-(terrain_tf.header.stamp.sec*10**9+terrain_tf.header.stamp.nanosec))/1e9
        if 1<=tx<79 and 1<=ty<79:
            cell['terrain_cell']=[tx,ty]
            cell['terrain_layers_center']={n:float(a[ty,tx]) for n,a in layers.items()}
            cell['terrain_traversability_3x3']=layers['traversability'][ty-1:ty+2,tx-1:tx+2].tolist()
            cell['terrain_unknown_3x3']=layers['unknown'][ty-1:ty+2,tx-1:tx+2].tolist()
        for name,(arr,meta) in clouds.items():
            if meta['frame']!=grid.header.frame_id: continue
            near=arr[np.linalg.norm(arr[:,:2]-[wx,wy],axis=1)<.4]
            cell['cloud_nearby'][name]=dict(count=len(near),z_min=float(near[:,2].min()) if len(near) else None,z_max=float(near[:,2].max()) if len(near) else None)
        cells.append(cell)
    report['curves'].append(dict(index=k,lethal_interior_cells=cells))
out=root/'v9/blocked_forward_cells.json'
out.write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
