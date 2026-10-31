import json, math, time
from pathlib import Path
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from nav2_msgs.action import FollowPath
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from std_msgs.msg import String

def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def wrap(x): return math.atan2(math.sin(x),math.cos(x))
rclpy.init(); node=rclpy.create_node('fixed_followpath_probe_20261007'); qos=QoSProfile(depth=100)
data={'odom':[],'native':[],'feedback':[]}
def odom_cb(m): data['odom'].append({'t':time.monotonic(),'xy':[m.pose.pose.position.x,m.pose.pose.position.y],'yaw':yaw(m.pose.pose.orientation),'stamp':[m.header.stamp.sec,m.header.stamp.nanosec]})
def native_cb(m):
 try:data['native'].append({'t':time.monotonic(),**json.loads(m.data)})
 except Exception:pass
node.create_subscription(Odometry,'/spot1/odom',odom_cb,qos)
node.create_subscription(String,'/wildos/native_control_status',native_cb,qos)
pub=node.create_publisher(Twist,'/wildos/cmd_vel',qos)
client=node.create_client(SetParameters,'/wildos_kinematic_vehicle_control/set_parameters')
def spin(sec):
 end=time.monotonic()+sec
 while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.025)
def enable(value):
 assert client.wait_for_service(timeout_sec=5)
 req=SetParameters.Request();req.parameters=[Parameter(name='enable',value=ParameterValue(type=ParameterType.PARAMETER_BOOL,bool_value=value))]
 f=client.call_async(req);end=time.monotonic()+5
 while not f.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
 assert f.done() and f.result().results[0].successful
act=ActionClient(node,FollowPath,'/follow_path'); handle=None; future=None; report={'test':'fixed_straight_followpath_3m','map_reload_count':0}
try:
 enable(True);spin(4)
 assert data['odom'] and data['native'],'missing odom/control status'
 assert data['native'][-1].get('telemetry_ok') and data['native'][-1].get('profile')=='all-fixed-v1','actuator not healthy'
 origin=data['odom'][-1]; h=origin['yaw']; report['origin']=origin
 assert act.wait_for_server(timeout_sec=5),'FollowPath action server unavailable'
 path=NavPath();path.header.frame_id='spot1/odom';path.header.stamp=node.get_clock().now().to_msg()
 points=[]
 for i in range(31):
  d=i*.1;p=PoseStamped();p.header=path.header;p.pose.position.x=origin['xy'][0]+d*math.cos(h);p.pose.position.y=origin['xy'][1]+d*math.sin(h);p.pose.orientation.z=math.sin(h/2);p.pose.orientation.w=math.cos(h/2);path.poses.append(p);points.append([p.pose.position.x,p.pose.position.y])
 goal=FollowPath.Goal();goal.path=path;goal.controller_id='FollowPath';goal.goal_checker_id='maneuver_goal_checker'
 start=time.monotonic();f0=len(data['odom'])
 def feedback_cb(f):data['feedback'].append({'t':time.monotonic(),'distance':f.feedback.distance_to_goal,'speed':f.feedback.speed})
 send=act.send_goal_async(goal,feedback_callback=feedback_cb);end=time.monotonic()+8
 while not send.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
 assert send.done(),'FollowPath goal response timeout'
 handle=send.result();report['accepted']=bool(handle.accepted);assert handle.accepted,'FollowPath rejected goal'
 future=handle.get_result_async();end=time.monotonic()+15
 while not future.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
 if not future.done():
  cf=handle.cancel_goal_async();end=time.monotonic()+3
  while not cf.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
  report['timeout']=True
 else:
  res=future.result();report['status']=res.status;report['result']=str(res.result);report['succeeded']=res.status==GoalStatus.STATUS_SUCCEEDED
 report['duration_s']=time.monotonic()-start;report['feedback']=data['feedback'];report['feedback_count']=len(data['feedback'])
 last=data['odom'][-1];report['end_pose']=last;report['endpoint_error_m']=math.hypot(last['xy'][0]-points[-1][0],last['xy'][1]-points[-1][1])
 rows=data['odom'][f0:]
 def error(p):
  a=points[0];b=points[-1];dx=b[0]-a[0];dy=b[1]-a[1];t=max(0,min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy)));return math.hypot(p[0]-(a[0]+t*dx),p[1]-(a[1]+t*dy))
 report['max_cross_track_m']=max((error(r['xy']) for r in rows),default=0);report['odom_samples']=rows
 if report.get('succeeded'):
  # Restore the initial position with the same ideal-point actuator after the action.
  p0=origin['xy'];deadline=time.monotonic()+8
  while time.monotonic()<deadline:
   cur=data['odom'][-1];dx=cur['xy'][0]-p0[0];dy=cur['xy'][1]-p0[1];along=dx*math.cos(h)+dy*math.sin(h);cross=-dx*math.sin(h)+dy*math.cos(h)
   if math.hypot(along,cross)<.025:break
   v=-math.copysign(min(1.2,max(.15,abs(along))),along)
   m=Twist();m.linear.x=v;pub.publish(m);rclpy.spin_once(node,timeout_sec=.025)
  for _ in range(10):pub.publish(Twist());rclpy.spin_once(node,timeout_sec=.025)
  spin(.5);report['return_residual_m']=math.hypot(data['odom'][-1]['xy'][0]-p0[0],data['odom'][-1]['xy'][1]-p0[1])
except BaseException as e:report['error']=repr(e)
finally:
 if handle and future and not future.done():
  try:handle.cancel_goal_async()
  except Exception:pass
 for _ in range(10):pub.publish(Twist());rclpy.spin_once(node,timeout_sec=.025)
 try:enable(False);report['disabled_after']=True
 except Exception as e:report['disabled_after']=False;report['disable_error']=repr(e)
 report['native_last']=data['native'][-1] if data['native'] else None
 Path('/home/ly/town_memory_validation_20261009/obstacle_restore/proxy_revision/followpath_probe.json').write_text(json.dumps(report,indent=2))
 node.destroy_node();rclpy.shutdown()
print(json.dumps({k:report.get(k) for k in ('test','accepted','succeeded','status','duration_s','feedback_count','endpoint_error_m','max_cross_track_m','return_residual_m','disabled_after','error','native_last')},separators=(',',':')))
