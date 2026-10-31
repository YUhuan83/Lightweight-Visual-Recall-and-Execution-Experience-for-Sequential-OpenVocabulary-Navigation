"""Read selected geometry from this closed trial without contacting ROS."""
from pathlib import Path
import hashlib
import json
import math
import sqlite3
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Path as NavPath, Odometry
from std_msgs.msg import String

root=Path('/home/ly/execution_continuity_repair_20261021/reachable_observation_v11/full/trial_20261021_221131')
goal=root/'sessions/session_20261021_221131/goal_001'
bag=next((goal/'interfaces').glob('*.db3'))
types={'/wildos/planned_path':NavPath,'/wildos/full_planned_path':NavPath,
       '/spot1/graphnav_planner/path':NavPath,'/spot1/odom':Odometry,
       '/spot1/graphnav_planner/route_status':String,
       '/spot1/graphnav_planner/retired_exploration':String}
def pose(p):
    q=p.orientation
    return [p.position.x,p.position.y,
            math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
rows=[]
with sqlite3.connect(bag.resolve().as_uri()+'?mode=ro',uri=True) as db:
    marks=','.join('?' for _ in types)
    for topic,ns,data in db.execute('SELECT t.name,m.timestamp,m.data FROM messages m JOIN topics t ON t.id=m.topic_id WHERE t.name IN ('+marks+') ORDER BY m.timestamp',tuple(types)):
        msg=deserialize_message(data,types[topic])
        row=dict(topic=topic,bag_ns=ns)
        if types[topic] is String:
            row['data']=json.loads(msg.data)
        else:
            row.update(stamp_ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                       frame=msg.header.frame_id)
            if types[topic] is Odometry:
                row.update(pose=pose(msg.pose.pose),speed=math.hypot(msg.twist.twist.linear.x,msg.twist.twist.linear.y))
            else:row['poses']=[pose(p.pose) for p in msg.poses]
        rows.append(row)
out=root/'fluency_audit/path_geometry.json'
out.write_text(json.dumps(dict(bag_sha256=hashlib.sha256(bag.read_bytes()).hexdigest(),rows=rows)))
print(json.dumps(dict(path=str(out),bytes=out.stat().st_size,rows=len(rows))))
