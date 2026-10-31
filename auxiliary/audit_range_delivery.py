import json,sqlite3,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from visual_navigation.utils.semantic_mask import largest_mask_component
base=Path('/home/ly/execution_continuity_review_20261022');root=base/'v47_depth_geometry';r=json.loads((root/'field_result.json').read_text());assert r['result']['closed'] and not r['postcheck']['processes']
g=next(Path(r['trial']).glob('sessions/*/goal_001'));db=sqlite3.connect(next((g/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={i:(n,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')};ids=[i for i,(n,_) in topics.items() if n in ('/spot1/object_mask','/spot1/object_localization')]
mask=None;rows=[]
for i,t,b in db.execute('select topic_id,timestamp,data from messages where topic_id in (?,?) order by timestamp,id',ids):
 name,c=topics[i];m=deserialize_message(b,c)
 if name=='/spot1/object_mask':mask=m;continue
 d=json.loads(m.data)
 if not d.get('localization_valid') or mask is None:continue
 tf=mask.cam_transforms.transforms[0];q=tf.transform.rotation;p=tf.transform.translation;rotation=R.from_quat([q.x,q.y,q.z,q.w]).as_matrix();camera=(np.asarray(d['position'])-[p.x,p.y,p.z])@rotation
 info=mask.cam_infos[0];k=np.asarray(info.k).reshape(3,3);h=k@camera;uv=h[:2]/h[2];u,v=np.floor(uv).astype(int)
 shape=[a.size for a in mask.object_mask.layout.dim];binary=largest_mask_component(np.asarray(mask.object_mask.data,dtype=np.uint8).reshape(shape).reshape(info.height,info.width).astype(bool));ms=mask.header.stamp.sec*10**9+mask.header.stamp.nanosec
 rows.append(dict(stamp_ns=d['observation_stamp_ns'],latest_mask_ns=ms,mask_delta_s=(ms-d['observation_stamp_ns'])*1e-9,receive_age_s=(t-d['observation_stamp_ns'])*1e-9,pixel=uv.tolist(),inside_latest=bool(camera[2]>.1 and 0<=u<info.width and 0<=v<info.height and binary[v,u])))
db.close();(base/'v48_range_candidate/range_delivery_audit.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows,indent=2))
