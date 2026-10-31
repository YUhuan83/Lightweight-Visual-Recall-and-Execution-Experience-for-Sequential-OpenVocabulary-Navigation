"""Inspect recorded mask components and project existing geometry into later masks."""
import hashlib,json,sqlite3
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
root=Path('/home/ly/execution_continuity_review_20261022/v28_intent_handoff')
trial=Path(json.loads((root/'field_result.json').read_text())['trial'])
assert json.loads((trial/'result.json').read_text())['closed']
goal=next(trial.glob('sessions/*/goal_001'))
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,t) for i,n,t in db.execute('select id,name,type from topics')}
def messages(name):
    tid,typ=topics[name];cls=get_message(typ)
    return [(t,deserialize_message(b,cls)) for t,b in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid,))]
localizations=[json.loads(m.data) for _,m in messages('/spot1/object_localization')]
masks=messages('/spot1/object_mask');db.close()
rows=[];raw_masks=[];rotations=[];origins=[];intrinsics=[];stamps=[];body_positions=[]
for recorded,m in masks:
    stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
    p=m.odom.pose.pose.position;body_positions.append([p.x,p.y,p.z])
    dims=[d.size for d in m.object_mask.layout.dim]
    masks=np.array(m.object_mask.data[m.object_mask.layout.data_offset:],dtype=np.uint8).reshape(dims)[:,0]
    cameras=[]
    for i,mask in enumerate(masks):
        count,labels,stats,_=cv2.connectedComponentsWithStats(mask,8)
        selected=1+int(np.argmax(stats[1:,cv2.CC_STAT_AREA])) if count>1 else 0
        tf=m.cam_transforms.transforms[i].transform;q=tf.rotation;t=tf.translation
        rotation=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();origin=np.array([t.x,t.y,t.z])
        k=np.array(m.cam_infos[i].k).reshape(3,3)
        projected=[]
        for j,data in enumerate(localizations):
            cam=rotation.T@(np.array(data['position'])-origin);label=0;uv=None
            if cam[2]>.1:
                p=k@cam;uv=(p[:2]/p[2]).tolist();u,v=uv
                if 0<=u<mask.shape[1] and 0<=v<mask.shape[0]:label=int(labels[int(v),int(u)])
            projected.append(dict(anchor=j,stamp_ns=data['observation_stamp_ns'],dt_s=(stamp-data['observation_stamp_ns'])/1e9,
                uv=uv,label=label,inside_selected=bool(label and label==selected)))
        components=[]
        for label in sorted(range(1,count),key=lambda label:-stats[label,cv2.CC_STAT_AREA])[:8]:
            y,x=np.nonzero(labels==label);v,u=np.median(np.column_stack([y,x]),axis=0)
            ray=rotation@np.linalg.solve(k,[u,v,1.])
            components.append(dict(label=int(label),area=int(stats[label,cv2.CC_STAT_AREA]),
                bbox=stats[label,:4].tolist(),world_bearing=float(np.arctan2(ray[1],ray[0]))))
        cameras.append(dict(index=i,selected=selected,components=components,projected_anchors=projected))
        if i==0:raw_masks.append(mask);rotations.append(rotation);origins.append(origin);intrinsics.append(k);stamps.append(stamp)
    rows.append(dict(stamp_ns=stamp,recorded_ns=recorded,cameras=cameras))
out=dict(trial=str(trial),localizations=localizations,rows=rows,
    caveat='Projection agreement is not semantic ground truth, independent range, or arrival confirmation.')
encoded=json.dumps(out,sort_keys=True).encode();(root/'mask_identity_audit.json').write_bytes(encoded)
np.savez_compressed(root/'mask_identity_fixture.npz',masks=raw_masks,rotations=rotations,origins=origins,intrinsics=intrinsics,stamps=stamps,body_positions=body_positions)
print(json.dumps(dict(sha256=hashlib.sha256(encoded).hexdigest(),masks=len(rows),
 multi_component=sum(len(r['cameras'][0]['components'])>1 for r in rows),
 fresh_other_component=[dict(stamp=r['stamp_ns'],anchor=p['anchor'],dt=p['dt_s']) for r in rows for c in r['cameras']
    for p in c['projected_anchors'] if 0<p['dt_s']<=5 and p['label'] and not p['inside_selected']])))
