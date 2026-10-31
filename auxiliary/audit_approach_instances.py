"""CLOSED evidence only: producer instance transitions and exact camera projection."""
import bisect,collections,hashlib,json,math,sqlite3,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
root=Path('/home/ly/execution_continuity_review_20261022')/sys.argv[1]
r=json.loads((root/'field_result.json').read_text());assert r['result']['closed'] and not r['postcheck']['processes']
g=next(Path(r['trial']).glob('sessions/*/goal_001'))
db=sqlite3.connect(next((g/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
def rows(name):
    i,cls=topics[name]
    return [(t,deserialize_message(blob,cls)) for t,blob in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(i,))]
confirm=[dict(received_ns=t,**json.loads(m.data)) for t,m in rows('/spot1/object_confirmation')]
loc=[dict(received_ns=t,**json.loads(m.data)) for t,m in rows('/spot1/object_localization')]
ms=rows('/spot1/object_mask');rgb=json.loads((g/'result.json').read_text())['failure_image']['observation_stamp_ns']
def ns(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
_,m=min(ms,key=lambda row:abs(ns(row[1])-rgb))
tf=m.cam_transforms.transforms[0];k=np.asarray(m.cam_infos[0].k).reshape(3,3)
q=tf.transform.rotation;p=tf.transform.translation
rot=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();origin=np.asarray([p.x,p.y,p.z])
shape=[d.size for d in m.object_mask.layout.dim];mask=np.asarray(m.object_mask.data).reshape(shape)
assert int(np.prod(shape[:-2]))==1,shape
mask=mask.reshape(shape[-2:])
odoms=rows('/spot1/odom')
rgb_odom=min(odoms,key=lambda row:abs(ns(row[1])-rgb))[1]
mask_odom=min(odoms,key=lambda row:abs(ns(row[1])-ns(m)))[1]
def pose_data(odom):
    p=odom.pose.pose.position;q=odom.pose.pose.orientation
    return np.asarray([p.x,p.y,p.z]),Rotation.from_quat([q.x,q.y,q.z,q.w])
pa,ra=pose_data(rgb_odom);pb,rb=pose_data(mask_odom)
extrinsic_rotation=rb.as_matrix().T@rot
extrinsic_translation=rb.as_matrix().T@(origin-pb)
rgb_rotation=ra.as_matrix()@extrinsic_rotation
rgb_origin=pa+ra.as_matrix()@extrinsic_translation
extrinsic_checks=[]
for _,other in ms[-5:]:
    body=min(odoms,key=lambda row:abs(ns(row[1])-ns(other)))[1]
    position,orientation=pose_data(body);other_tf=other.cam_transforms.transforms[0]
    oq=other_tf.transform.rotation;op=other_tf.transform.translation
    observed_rotation=Rotation.from_quat([oq.x,oq.y,oq.z,oq.w]).as_matrix()
    observed_origin=np.asarray([op.x,op.y,op.z])
    er=orientation.as_matrix().T@observed_rotation
    et=orientation.as_matrix().T@(observed_origin-position)
    extrinsic_checks.append(dict(translation_m=float(np.linalg.norm(et-extrinsic_translation)),
        rotation_rad=float(Rotation.from_matrix(extrinsic_rotation.T@er).magnitude())))
positive=[d for d in confirm if d.get('search_confirmed')]
def project(point):
    camera=rot.T@(np.asarray(point)-origin);pixel=k@camera
    xy=(pixel[:2]/pixel[2]).tolist();u,v=map(int,xy)
    rgb_camera=rgb_rotation.T@(np.asarray(point)-rgb_origin);rgb_h=k@rgb_camera
    return dict(pixel=xy,rgb_pixel=(rgb_h[:2]/rgb_h[2]).tolist(),depth_m=float(camera[2]),inside_image=0<=v<mask.shape[0] and 0<=u<mask.shape[1],
        inside_mask=bool(mask[v,u]) if 0<=v<mask.shape[0] and 0<=u<mask.shape[1] else False)
points=[]
for tag,data in [('first_confirmed',positive[0]),('last_confirmed',positive[-1]),('latest_localized',loc[-1])]:
    pos=data.get('object_position',data.get('position'))
    points.append(dict(tag=tag,point=pos,observation_stamp_ns=data['observation_stamp_ns'],instance_id=data.get('instance_id'),projection=project(pos)))
db.close()
out=dict(trial=r['trial'],rgb_stamp_ns=rgb,mask_stamp_ns=ns(m),mask_rgb_dt_s=(ns(m)-rgb)*1e-9,
    frame=m.cam_infos[0].header.frame_id,width=m.cam_infos[0].width,height=m.cam_infos[0].height,mask_shape=shape,
    rgb_mask_pose_delta=dict(translation_m=float(np.linalg.norm(pa-pb)),rotation_rad=float((ra.inv()*rb).magnitude()),
        rgb_odom_age_s=abs(ns(rgb_odom)-rgb)*1e-9,mask_odom_age_s=abs(ns(mask_odom)-ns(m))*1e-9),
    fixed_camera_extrinsic_checks=extrinsic_checks,
    projected_points=points,confirmation_messages=confirm,localization_messages=loc,
    scope='Exact stored camera K/TF. Camera acquisition may differ from failure RGB; dt explicitly stated. Geometric pixel support is not semantic ground truth.',acceptance=False)
(root/'approach_instance_audit.json').write_text(json.dumps(out,indent=2))
print(json.dumps({k:v for k,v in out.items() if not k.endswith('_messages')},indent=2))
