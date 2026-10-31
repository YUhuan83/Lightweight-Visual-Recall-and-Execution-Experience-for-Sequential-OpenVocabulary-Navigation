"""Inspect only the closed 180945 bag; never starts ROS nodes or vehicle."""
from pathlib import Path
import json, math, sqlite3
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

trial=Path('/home/ly/execution_continuity_review_20261022/v4/full/trial_20261022_180945')
assert json.loads((trial/'result.json').read_text())['closed']
goal=next(trial.glob('sessions/*/goal_001'))
db=sqlite3.connect(next(goal.rglob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={name:(tid,typ) for tid,name,typ in db.execute('select id,name,type from topics')}
print(json.dumps({'topics':topics}),flush=True)
out=trial/'observation_replay';out.mkdir(exist_ok=True)
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def pose(p):return [p.position.x,p.position.y,yaw(p.orientation)]
request_ns=1792663964143512664
rows={}
for name,(tid,typ) in topics.items():
    if typ not in ('nav_msgs/msg/Odometry','nav_msgs/msg/OccupancyGrid','nav_msgs/msg/Path','tf2_msgs/msg/TFMessage'):
        continue
    entry=db.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',(tid,request_ns)).fetchone()
    if not entry:continue
    ts,data=entry;msg=deserialize_message(data,get_message(typ))
    record={'bag_stamp_ns':ts,'bag_age_sec':(request_ns-ts)/1e9,'frame':getattr(getattr(msg,'header',None),'frame_id',None)}
    if typ=='nav_msgs/msg/Odometry':record.update(pose=pose(msg.pose.pose),stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec)
    elif typ=='nav_msgs/msg/Path':record.update(poses=len(msg.poses),ends=[pose(p.pose) for p in (msg.poses[:1]+msg.poses[-2:])])
    elif typ=='tf2_msgs/msg/TFMessage':record['transforms']=[{'frame':t.header.frame_id,'child':t.child_frame_id,'xy':[t.transform.translation.x,t.transform.translation.y],'yaw':yaw(t.transform.rotation),'stamp_ns':t.header.stamp.sec*10**9+t.header.stamp.nanosec} for t in msg.transforms]
    else:
        record.update(size=[msg.info.width,msg.info.height],resolution=msg.info.resolution,origin=pose(msg.info.origin))
        if 'global_costmap' in name:
            path=out/'request59_public_grid.json'
            path.write_text(json.dumps({'name':name,'record':record,'data':list(msg.data)}))
    rows[name]=record
(out/'request59_interfaces.json').write_text(json.dumps(rows,indent=2))
print(json.dumps(rows,indent=2),flush=True)
for name,(tid,typ) in topics.items():
    if typ=='nav_msgs/msg/Odometry':
        window=[]
        for ts,data in db.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp',(tid,request_ns-200_000_000,request_ns+700_000_000)):
            msg=deserialize_message(data,get_message(typ));window.append({'delta_s':(ts-request_ns)/1e9,'pose':pose(msg.pose.pose)})
        print(json.dumps({'name':name,'odom_window':window}),flush=True)
