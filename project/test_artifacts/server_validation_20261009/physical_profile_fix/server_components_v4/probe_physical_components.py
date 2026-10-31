import json,math,time,pathlib,yaml,copy
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from rcl_interfaces.srv import GetParameters
from nav_msgs.msg import Odometry,OccupancyGrid
from sensor_msgs.msg import Image,PointCloud2,CameraInfo
from std_msgs.msg import String
from nav2_msgs.action import ComputePathToPose
from wildos_nav2_adapter.runtime_contract import navigation_runtime_contract
rclpy.init();n=Node('physical_component_read_only_probe');seen={};counts={};errors=[];actual={}
def cb(k):
 def f(m):seen[k]=m;counts[k]=counts.get(k,0)+1
 return f
for typ,topic,k in [(Odometry,'/spot1/odom','odom'),(Image,'/spot1/realsense/front/color/image_raw','image'),(CameraInfo,'/spot1/realsense/front/color/camera_info','camera_info'),(PointCloud2,'/wildos/points','obstacles'),(PointCloud2,'/wildos/clearing_points','clearing'),(OccupancyGrid,'/local_costmap/costmap','local'),(OccupancyGrid,'/global_costmap/costmap','global'),(String,'/wildos/native_control_status','native')]:n.create_subscription(typ,topic,cb(k),qos_profile_sensor_data)
t=time.monotonic()
while time.monotonic()-t<5:rclpy.spin_once(n,timeout_sec=.1)
config=yaml.safe_load(pathlib.Path('nav2_adapter/config/nav2_ackermann_control.yaml').read_text())
required=navigation_runtime_contract(config,'ackermann');required['/wildos_cmd_vel_to_control']['enable']=False;required['/wildos_graph_path_bridge']['follow_path']=False
for owner,fields in required.items():
 client=n.create_client(GetParameters,owner+'/get_parameters')
 if not client.wait_for_service(timeout_sec=2):errors.append(owner+' unavailable');continue
 future=client.call_async(GetParameters.Request(names=list(fields)));rclpy.spin_until_future_complete(n,future,timeout_sec=3)
 if not future.done() or future.exception():errors.append(owner+' query failed');continue
 values={}
 for (key,expected),value in zip(fields.items(),future.result().values):
  attr={1:'bool_value',2:'integer_value',3:'double_value',4:'string_value',9:'string_array_value'}.get(value.type)
  observed=getattr(value,attr) if attr else None
  if value.type==9:observed=list(observed)
  values[key]=observed
  if observed!=expected:errors.append(owner+'/'+key+' differs')
 actual[owner]=values;n.destroy_client(client)
for k in ('odom','image','camera_info','obstacles','clearing','local','global','native'):
 if counts.get(k,0)<2:errors.append(k+' missing/insufficient')
report=dict(scope='Stationary live parameter, sensor, costmap and ComputePathToPose only; no FollowPath or motion',actual_parameters=actual,counts=counts,errors=errors)
for k in ('local','global'):
 if k in seen:
  m=seen[k];report[k]=dict(frame=m.header.frame_id,resolution=m.info.resolution,width=m.info.width,height=m.info.height,occupied=sum(v>=99 for v in m.data),stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec)
if 'native' in seen:report['native']=json.loads(seen['native'].data)
if 'odom' in seen:
 pose=seen['odom'].pose.pose;client=ActionClient(n,ComputePathToPose,'/compute_path_to_pose')
 q=pose.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));g=ComputePathToPose.Goal();g.planner_id='CarLike';g.use_start=False;g.goal.header.frame_id='spot1/odom';g.goal.header.stamp=n.get_clock().now().to_msg();g.goal.pose=copy.deepcopy(pose);g.goal.pose.position.x+=3*math.cos(yaw);g.goal.pose.position.y+=3*math.sin(yaw)
 report['requested_goal']=[g.goal.pose.position.x,g.goal.pose.position.y,yaw]
 if client.wait_for_server(timeout_sec=3):
  f=client.send_goal_async(g);rclpy.spin_until_future_complete(n,f,timeout_sec=5)
  if f.done() and not f.exception() and f.result().accepted:
   f=f.result().get_result_async();rclpy.spin_until_future_complete(n,f,timeout_sec=8)
   if f.done() and not f.exception():
    result=f.result();report['plan_status']=result.status;report['path']=[[p.pose.position.x,p.pose.position.y,p.pose.orientation.z,p.pose.orientation.w] for p in result.result.path.poses]
   else:errors.append('plan result timeout/error')
  else:errors.append('plan rejected/timeout')
 else:errors.append('planner action unavailable')
 if report.get('plan_status')!=4 or len(report.get('path',[]))<2:errors.append('CarLike did not produce successful path')
else:errors.append('odometry absent for planning')
report['passed']=not errors
p=pathlib.Path('../physical_components_v1.json');assert not p.exists();p.write_text(json.dumps(report,indent=2));print(json.dumps({k:report.get(k) for k in ('passed','errors','counts','plan_status','local','global')}));n.destroy_node();rclpy.shutdown()
