"""Read closed-bag RGB and exact-stamp obstacle returns at approach abort."""
import json
import math
from pathlib import Path
import sqlite3
import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.pointcloud import decode_xyz
from wildos_nav2_adapter.math3d import matrix_from_quaternion
from wildos_nav2_adapter.vehicle_geometry import SBR_BASE_OFFSET
from wildos_nav2_adapter.graph_route import cell_cost

root = Path(__file__).resolve().parent
trial = root / 'full/trial_20261029_212751'
assert json.loads((trial/'result.json').read_text())['closed']
goal = next(trial.glob('sessions/*/goal_001'))
events = [json.loads(line) for line in (goal/'events.jsonl').read_text().splitlines()]
query = next(e['elapsed'] for e in events if e.get('kind') == 'query_sent')
event = next(e for e in events if e.get('status') == 'follow_failed' and e['elapsed']-query > 80)
stamp = event['stamp_ns']
db = sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro', uri=True)
topics = {n:(i,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
def before(name, t):
    i, kind = topics[name]
    row = db.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',(i,t)).fetchone()
    return row[0], deserialize_message(row[1], kind)
def ns(message):return message.header.stamp.sec*10**9+message.header.stamp.nanosec
odoms={ns(o):o for (data,) in db.execute('select data from messages where topic_id=?',(topics['/spot1/odom'][0],))
       for o in [deserialize_message(data,topics['/spot1/odom'][1])]}
rgb_topic='/spot1/realsense/front/color/image_raw'
rgb_age=None
if rgb_topic in topics:
    rt, image=before(rgb_topic,stamp)
    rgb_age=(stamp-rt)*1e-9
    rgb=np.frombuffer(image.data,dtype=np.uint8).reshape(image.height,image.step)[:,:image.width*3].reshape(image.height,image.width,3)
    if image.encoding.lower()=='rgb8':rgb=cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)
    cv2.imwrite(str(trial/'approach_abort_rgb.png'),rgb)
mt, local=before('/local_costmap/costmap',stamp)
gt, global_map=before('/global_costmap/costmap',stamp)
rt, msg=before('/wildos/points',stamp)
odom=odoms[ns(msg)]
q=odom.pose.pose.orientation;pose=odom.pose.pose.position
r=np.array(matrix_from_quaternion([q.x,q.y,q.z,q.w]));base=np.array([pose.x,pose.y,pose.z])
p=decode_xyz(msg)
assert msg.header.frame_id in ('spot1/base_link','spot1/odom')
world=p@r.T+base if msg.header.frame_id=='spot1/base_link' else p
# The first imminent path-footprint blocker in the existing audit.
audit=json.loads((trial/'tracking_collision_audit.json').read_text())[0]
blocked=audit['path_blocked_within4m'][0]
x,y,h=blocked['pose'];front,left=blocked['local'][0]
hit=np.array([x+math.cos(h)*front-math.sin(h)*left,y+math.sin(h)*front+math.cos(h)*left])
near=np.linalg.norm(world[:,:2]-hit,axis=1)<.6
cloud_rows=[]
for point in world[near]:
    cloud_rows.append(dict(world=point.tolist(),local_cost=cell_cost(local,*point[:2]),global_cost=cell_cost(global_map,*point[:2])))
rawt, raw=before('/spot1/ouster/front/points_filtered',stamp)
rawpose=odoms[ns(raw)].pose.pose
q=rawpose.orientation;rawr=np.array(matrix_from_quaternion([q.x,q.y,q.z,q.w]));v=rawpose.position
rawpoints=decode_xyz(raw)-np.asarray(SBR_BASE_OFFSET)
rawworld=rawpoints@rawr.T+np.array([v.x,v.y,v.z])
patch=rawworld[np.linalg.norm(rawworld[:,:2]-hit,axis=1)<1.]
report=dict(query_s=event['elapsed']-query,rgb_receipt_age_s=rgb_age,
            blocker_world_approx=hit.tolist(),mark_frame=msg.header.frame_id,
            marks_receipt_age_s=(stamp-rt)*1e-9,local_map_receipt_age_s=(stamp-mt)*1e-9,
            global_map_receipt_age_s=(stamp-gt)*1e-9,nearby_current_markings=cloud_rows,
            raw_patch_world=patch.tolist())
(trial/'abort_obstacle_source.json').write_text(json.dumps(report,indent=2))
db.close()
print(json.dumps({k:v for k,v in report.items() if k!='raw_patch_world'},indent=2))
