"""Closed-recording mask projection of the selected and replacement anchors.

Mask overlap is geometric association evidence, not RGB semantic ground truth.
"""
import json,sqlite3,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from visual_navigation.utils.semantic_mask import largest_mask_component

root=Path('/home/ly/execution_continuity_review_20261022/v54_confirmed_graph_handoff')
receipt=json.loads((root/'field_result.json').read_text())
assert receipt['result']['closed'] and not receipt['postcheck']['processes'] and not receipt['postcheck']['display148_lock']
goal=next(Path(receipt['trial']).glob('sessions/*/goal_001'))
events=[json.loads(line) for line in (goal/'events.jsonl').read_text().splitlines()]
query_elapsed=next(e['elapsed'] for e in events if e.get('kind')=='query_sent')
offset=next(e['stamp_ns']*1e-9-(e['elapsed']-query_elapsed) for e in events if e.get('stamp_ns'))
superseded=next(e for e in events if e.get('status')=='target_instance_superseded')
output=root/'instance_projection_audit';output.mkdir(exist_ok=True)
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
def messages(name):
 i,c=topics[name]
 for t,b in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(i,)):
  yield t,deserialize_message(b,c)
def ns(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
confirmations=[json.loads(m.data) for _,m in messages('/spot1/object_confirmation')]
valid=[c for c in confirmations if c.get('search_confirmed') and c.get('object_position')]
first=next(c for c in valid if c['instance_id']==superseded['instance_id'])
later=next(c for c in valid if c['instance_id']!=superseded['instance_id'])
localizations={d['observation_stamp_ns']:d for _,m in messages('/spot1/object_localization') for d in [json.loads(m.data)] if d.get('localization_valid')}
anchors=dict(selected=first['object_position'],replacement=superseded['replacement_candidate']['position'],later=later['object_position'])
rows=[]
for _,m in messages('/spot1/object_mask'):
 stamp=ns(m);shape=[int(d.size) for d in m.object_mask.layout.dim]
 masks=np.asarray(m.object_mask.data[m.object_mask.layout.data_offset:],dtype=np.uint8).reshape(shape)[:,0]
 selected=[largest_mask_component(mask.astype(bool)) for mask in masks]
 camera=max(range(len(selected)),key=lambda i:np.count_nonzero(selected[i]));mask=selected[camera]
 tf=m.cam_transforms.transforms[camera];info=m.cam_infos[camera]
 assert tf.header.frame_id=='spot1/odom' and tf.child_frame_id.lstrip('/')==info.header.frame_id.lstrip('/')
 assert abs(ns(tf)-stamp)<=50_000_000 and abs(ns(info)-stamp)<=50_000_000
 q=tf.transform.rotation;t=tf.transform.translation
 rotation=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();origin=np.asarray([t.x,t.y,t.z]);k=np.asarray(info.k).reshape(3,3)
 positions=dict(anchors)
 if stamp in localizations:positions['current_measured']=localizations[stamp]['position']
 projection={}
 for name,p in positions.items():
  camera_point=(np.asarray(p)-origin)@rotation
  item=dict(depth_m=float(camera_point[2]),in_image=False,in_query_mask=False,in_selected_component=False)
  if camera_point[2]>.01:
   uv=(k@camera_point)[:2]/camera_point[2];x,y=np.rint(uv).astype(int)
   item['pixel']=uv.tolist();item['in_image']=bool(0<=x<mask.shape[1] and 0<=y<mask.shape[0])
   if item['in_image']:item.update(in_query_mask=bool(masks[camera,y,x]),in_selected_component=bool(mask[y,x]))
  projection[name]=item
 row=dict(stamp_ns=stamp,query_s=stamp*1e-9-offset,query=m.query,selected_mask_pixels=int(mask.sum()),projection=projection)
 if stamp in localizations:row['localization_quality']=localizations[stamp].get('quality')
 rows.append(row)
 # Retain only a few compact mask/transform checkpoints for review and tests.
 if abs(stamp-first['observation_stamp_ns'])<1 or abs(stamp-superseded['replacement_candidate']['observation_stamp_ns'])<1 or abs(stamp-later['observation_stamp_ns'])<1:
  name=f'mask_{stamp}.npz';np.savez_compressed(output/name,mask=mask,raw_mask=masks[camera],rotation=rotation,origin=origin,intrinsic=k,positions=np.asarray(list(positions.values())))
  row['fixture']=name;row['position_names']=list(positions)
db.close()
report=dict(scope=__doc__,trial=receipt['trial'],anchors=anchors,selected_instance=first['instance_id'],replacement_stamp_ns=superseded['replacement_candidate']['observation_stamp_ns'],later_instance=later['instance_id'],rows=rows)
encoded=json.dumps(report,indent=2,sort_keys=True).encode();(output/'report.json').write_bytes(encoded)
print(json.dumps(dict(sha256=hashlib.sha256(encoded).hexdigest(),anchors=anchors,checkpoints=[r for r in rows if r.get('fixture')],recent=[r for r in rows if 120<=r['query_s']<=140]),indent=2))
