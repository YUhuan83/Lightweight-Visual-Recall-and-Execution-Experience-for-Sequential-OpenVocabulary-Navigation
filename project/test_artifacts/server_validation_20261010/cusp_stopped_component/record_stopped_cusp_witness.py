import rclpy,time,json,sys
from pathlib import Path
from collections import deque
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Path as Route,Odometry,OccupancyGrid
from std_msgs.msg import String
from wildos_nav2_adapter.graph_path_bridge import xy,yaw
from wildos_nav2_adapter.motion_path import motion_sections,diagnose_maneuver_join
s=Path('/home/ly/town_memory_validation_20261010/cusp_stopped_component1');f=(s.parent/'cusp_stopped_component_witness.jsonl').open('x');rclpy.init();n=rclpy.create_node('physical_cusp_witness');state={};plans=deque(maxlen=12)
def write(kind,**data):
 f.write(json.dumps(dict(kind=kind,receipt_ns=time.time_ns(),**data))+'\n');f.flush()
def plan(m):
 p=[xy(a.pose) for a in m.poses];h=[yaw(a.pose) for a in m.poses];item=dict(stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,points=p,headings=h,sections=motion_sections(p));plans.append(item);write('full_plan',**item)
def path(m):
 p=[xy(a.pose) for a in m.poses];matches=[a for a in plans if len(a['points'])>=len(p) and all(__import__('math').dist(x,y)<1e-6 for x,y in zip(p,a['points']))]
 state['selected']=matches[-1] if matches else None;write('sent_section',points=p,headings=[yaw(a.pose) for a in m.poses],full_plan_matches=len(matches),selected_plan_stamp=matches[-1]['stamp_ns'] if matches else None)
def odom(m):state['odom']=dict(position=xy(m.pose.pose),heading=yaw(m.pose.pose),linear_mps=m.twist.twist.linear.x,angular_radps=m.twist.twist.angular.z,stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec)
def status(m):
 d=json.loads(m.data)
 if d.get('status') in ('replanning_after_maneuver','maneuver_done','following','follow_failed','no_vehicle_path'):
  data=dict(status=d,actual=state.get('odom'),selected=state.get('selected'))
  if d['status']=='replanning_after_maneuver' and state.get('selected') and state.get('odom'):
   p=state['selected']; sections=p['sections']; actual=state['odom']
   if len(sections)>1:
    a,b=sections[1]; join,reason=diagnose_maneuver_join(p['points'][a:b+1],p['headings'][a:b+1],actual['position'],actual['heading'],8.);data.update(replayed_join_reason=reason,join=join)
  write('status',**data)
def grid(m,label):state[label]=dict(stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,origin=xy(m.info.origin),resolution=m.info.resolution,width=m.info.width,height=m.info.height,data=list(m.data))
def native(m):
 d=json.loads(m.data);state['native']=d
 if not hasattr(native,'last') or time.monotonic()-native.last>.2:write('native',payload=d);native.last=time.monotonic()
n.create_subscription(Route,'/plan',plan,10);n.create_subscription(Route,'/wildos/planned_path',path,10);n.create_subscription(Odometry,'/spot1/odom',odom,qos_profile_sensor_data);n.create_subscription(String,'/wildos/plan_status',status,10);n.create_subscription(String,'/wildos/native_control_status',native,10)
from geometry_msgs.msg import Twist
n.create_subscription(Twist,'/wildos/cmd_vel',lambda m:write('cmd_vel',linear_mps=m.linear.x,angular_radps=m.angular.z),10)
n.create_subscription(String,'/wildos/control_status',lambda m:write('control_status',payload=json.loads(m.data)),10)
deadline=time.monotonic()+120
try:
 while time.monotonic()<deadline and not (s/'result.json').exists():rclpy.spin_once(n,timeout_sec=.1)
finally:f.close();n.destroy_node();rclpy.shutdown()
