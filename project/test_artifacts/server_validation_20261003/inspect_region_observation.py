"""Project the remembered station point into later recorded semantic masks."""
from pathlib import Path
import json
import sqlite3
import sys
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

root=Path('/home/ly/town_memory_validation_20261003')/sys.argv[1]
events=[json.loads(x) for x in (root/'events.jsonl').read_text().splitlines()]
first=next(e['payload']['object_position'] for e in events
           if e['kind']=='object_confirmation' and e['phase']=='station_observation')
confirmations=[e for e in events if e['kind']=='object_confirmation' and e['phase']=='station_return']
db=sqlite3.connect(f"file:{next((root/'rosbag').glob('*.db3'))}?mode=ro",uri=True)
identifier,kind=db.execute('SELECT id,type FROM topics WHERE name=?',('/spot1/object_mask',)).fetchone()
kind=get_message(kind)
masks=[]
for ts,b in db.execute('SELECT timestamp,data FROM messages WHERE topic_id=?',(identifier,)):
    m=deserialize_message(b,kind)
    if m.query=='gas station':
        masks.append(m)
result=[]
for e in confirmations:
    stamp=e['payload']['observation_stamp_ns']
    m=min(masks,key=lambda m:abs(m.header.stamp.sec*10**9+m.header.stamp.nanosec-stamp))
    t=m.cam_transforms.transforms[0].transform
    r=Rotation.from_quat([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]).as_matrix()
    camera=(np.asarray(first)-[t.translation.x,t.translation.y,t.translation.z])@r
    projected=np.asarray(m.cam_infos[0].k).reshape(3,3)@camera
    pixel=projected[:2]/projected[2]
    dims=[d.size for d in m.object_mask.layout.dim]
    mask=np.asarray(m.object_mask.data).reshape(dims)[0,0]>0
    u,v=np.rint(pixel).astype(int)
    inside=camera[2]>.1 and 0<=u<mask.shape[1] and 0<=v<mask.shape[0]
    result.append({'elapsed':e['elapsed'],'object_position':e['payload']['object_position'],
        'distance_to_remembered_point':float(np.linalg.norm(np.asarray(first[:2])-e['payload']['object_position'][:2])),
        'remembered_point_camera':camera.tolist(),'projected_pixel':pixel.tolist(),
        'inside_image':bool(inside),'inside_positive_mask':bool(inside and mask[v,u])})
(root/'region_observation_diagnosis.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
