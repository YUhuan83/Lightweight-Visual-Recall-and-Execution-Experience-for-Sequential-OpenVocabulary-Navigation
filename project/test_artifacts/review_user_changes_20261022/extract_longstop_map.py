from pathlib import Path
import json, sqlite3, math
import numpy as np
import cv2
from rclpy.serialization import deserialize_message
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
root=Path('/home/ly/execution_continuity_review_20261022/v1/full/trial_20261022_155227')
goal=root/'sessions/session_20261022_155227/goal_001'
t0=1792655601.612708
db=sqlite3.connect(next(goal.rglob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics=dict(db.execute('select name,id from topics'))
out=root/'fluency_audit/longstop_maps';out.mkdir(exist_ok=True)
rows=[]
for elapsed in (260,265,275,350):
    now=int((t0+elapsed)*1e9)
    row={'elapsed':elapsed,'topics':{}}
    for name,cls in (('/spot1/local_terrain',GridMap),('/spot1/odom',Odometry),('/tf',TFMessage)):
        tid=topics[name]
        timestamp,data=db.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',(tid,now)).fetchone()
        msg=deserialize_message(data,cls)
        dest=out/(str(elapsed)+'_'+name.split('/')[-1]+'.cdr');dest.write_bytes(data)
        if cls is GridMap:
            row['topics'][name]={'frame':msg.header.frame_id,'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                'layers':list(msg.layers),'position':str(msg.info.pose),'resolution':msg.info.resolution,
                'length':[msg.info.length_x,msg.info.length_y],'outer':msg.outer_start_index,'inner':msg.inner_start_index,
                'dims':[[(d.label,d.size,d.stride) for d in a.layout.dim] for a in msg.data[:1]]}
        else:row['topics'][name]=str(msg)
    terrain=deserialize_message((out/(str(elapsed)+'_local_terrain.cdr')).read_bytes(),GridMap)
    transforms=[]
    targetstamp=terrain.header.stamp.sec+terrain.header.stamp.nanosec*1e-9
    for (data,) in db.execute('select data from messages where topic_id=? and timestamp between ? and ? order by timestamp',(topics['/tf'],int((targetstamp-2)*1e9),int((targetstamp+1)*1e9))):
        transforms.extend(t for t in deserialize_message(data,TFMessage).transforms if t.child_frame_id==terrain.header.frame_id)
    tf=min(transforms,key=lambda t:abs(t.header.stamp.sec+t.header.stamp.nanosec*1e-9-targetstamp))
    from rclpy.serialization import serialize_message
    (out/(str(elapsed)+'_terrain_tf.cdr')).write_bytes(serialize_message(tf))
    row['terrain_transform']=str(tf)
    layers={n:np.array(a.data).reshape(80,80)[::-1,::-1] for n,a in zip(terrain.layers,terrain.data)}
    sdf=cv2.distanceTransform((layers['traversability']<=.99).astype(np.uint8),cv2.DIST_L2,cv2.DIST_MASK_5)*.5
    q=tf.transform.rotation; yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    pos=tf.transform.translation
    locations={'actual':(735.905525,-44.897815),'graph_start':(736.2589517,-46.7630200)}
    row['clearance']={}
    for name,(x,y) in locations.items():
        dx=x-pos.x;dy=y-pos.y
        lx=dx*math.cos(yaw)+dy*math.sin(yaw);ly=-dx*math.sin(yaw)+dy*math.cos(yaw)
        ix=int(math.floor((lx+20)/.5));iy=int(math.floor((ly+20)/.5))
        row['clearance'][name]={'local':[lx,ly],'cell':[ix,iy], 'sdf':float(sdf[iy,ix]),
             'values':{n:float(a[iy,ix]) for n,a in layers.items()}}
    rows.append(row)
(out/'index.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows,indent=2))
