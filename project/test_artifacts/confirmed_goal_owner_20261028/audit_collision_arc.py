"""Compare last published arc and recorded local map near v58 controller abort."""
import json,sqlite3,math
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.graph_route import footprint_blockers
from wildos_nav2_adapter.graph_path_bridge import xy,yaw
trial=Path('/home/ly/execution_continuity_review_20261022/v58_confirmed_raw_handoff/full/trial_20261028_094705')
assert json.loads((trial/'result.json').read_text())['closed']
goal=next(trial.glob('sessions/*/goal_001'))
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
def before(name,stamp):
 i,c=topics[name]; row=db.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',(i,stamp)).fetchone()
 return (row[0],deserialize_message(row[1],c)) if row else (None,None)
out=[]
for t in [1793152166544698619,1793152203433633330]:
 at,a=before('/lookahead_collision_arc',t); mt,m=before('/local_costmap/costmap',t);ot,o=before('/spot1/odom',t)
 # nav2 publishes Point-derived poses without valid orientations: recover a
 # tangent estimate from neighboring XY and align its sign with vehicle yaw.
 blocked=[]; vehicle_yaw=yaw(o.pose.pose)
 for i,p in enumerate(a.poses):
  x0,y0=xy(a.poses[max(0,i-1)].pose);x1,y1=xy(a.poses[min(len(a.poses)-1,i+1)].pose)
  heading=math.atan2(y1-y0,x1-x0)
  if abs(math.remainder(heading-vehicle_yaw,2*math.pi))>math.pi/2:heading=math.remainder(heading+math.pi,2*math.pi)
  cells=footprint_blockers(m,xy(p.pose),heading,limit=4)
  if cells:blocked.append({'index':i,'position':[*xy(p.pose),heading],'cells':cells})
 out.append({'abort_stamp_ns':t,'arc_receipt_age_s':(t-at)*1e-9,'arc_header_ns':a.header.stamp.sec*1000000000+a.header.stamp.nanosec,'map_receipt_age_s':(t-mt)*1e-9,'map_header_ns':m.header.stamp.sec*1000000000+m.header.stamp.nanosec,'odom_receipt_age_s':(t-ot)*1e-9,'arc_frame':a.header.frame_id,'map_frame':m.header.frame_id,'arc_poses':len(a.poses),'first_orientation':[a.poses[0].pose.orientation.x,a.poses[0].pose.orientation.y,a.poses[0].pose.orientation.z,a.poses[0].pose.orientation.w] if a.poses else None,'arc_start':list(xy(a.poses[0].pose)) if a.poses else None,'arc_end':list(xy(a.poses[-1].pose)) if a.poses else None,'vehicle_pose':[*xy(o.pose.pose),vehicle_yaw],'approx_arc_blocked':blocked[:5]})
db.close()
(trial/'collision_arc_audit.json').write_text(json.dumps(out,indent=2))
print(json.dumps(out,indent=2))
