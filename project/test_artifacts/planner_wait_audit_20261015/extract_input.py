from pathlib import Path
import sqlite3,math,json
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
s=Path('/home/ly/wildos-console/audit_results/session_20261015_203918');assert (s/'session_result.json').exists() and (s/'goal_001/result.json').exists()
c=sqlite3.connect(next((s/'goal_001/interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,get_message(t)) for i,n,t in c.execute('select id,name,type from topics')};ns=1792068106906990077
previous=lambda name:c.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',(topics[name][0],ns)).fetchone()
ot,b=previous('/spot1/odom');odom=deserialize_message(b,topics['/spot1/odom'][1]);mt,b=previous('/global_costmap/costmap');m=deserialize_message(b,topics['/global_costmap/costmap'][1])
tid,kind=topics['/spot1/graphnav_planner/path'];goal=None;rt=None
for ts,b in c.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc',(tid,ns)):
 p=deserialize_message(b,kind)
 if p.poses and math.hypot(p.poses[-1].pose.position.x-722.5771919873481,p.poses[-1].pose.position.y+15.698726615670637)<1e-5:goal=p.poses[-1].pose;rt=ts;break
assert goal
p=odom.pose.pose;yaw=lambda q:math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
values=[m.info.width,m.info.height,m.info.resolution,m.info.origin.position.x,m.info.origin.position.y,p.position.x,p.position.y,yaw(p.orientation),goal.position.x,goal.position.y,yaw(goal.orientation)]
out=Path('/home/ly/planner_wait_audit_20261015');(out/'input.txt').write_text(' '.join(map(str,values))+'\n'+' '.join(map(str,m.data)))
(out/'input_scope.json').write_text(json.dumps(dict(values=values,request_wall_ns=ns,odom_receipt_ns=ot,map_receipt_ns=mt,route_receipt_ns=rt,scope='Nearest prior published odom/costmap and endpoint-matched graph route. Not private planner request or byte-exact costmap.'),indent=2));c.close()
