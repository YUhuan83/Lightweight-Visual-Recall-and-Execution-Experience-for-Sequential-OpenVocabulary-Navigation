import json,math,time,pathlib,copy,subprocess,signal,argparse
parser=argparse.ArgumentParser();parser.add_argument('--maneuver',choices=['left_arc','right_arc','reverse'],required=True);args=parser.parse_args()
import cv2,numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from rcl_interfaces.srv import GetParameters,SetParameters
from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType
from nav2_msgs.action import ComputePathToPose,FollowPath
from nav_msgs.msg import Odometry,Path,OccupancyGrid
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
out=pathlib.Path('/home/ly/town_memory_validation_20261010/'+args.maneuver);out.mkdir(exist_ok=False)
rclpy.init();n=Node('physical_follow_path_component');latest={};samples=[];frames=[];events=[];errors=[];handle=None
started=time.monotonic();image=None;written=0;grid_history=[];timing_history=[];tf_history=[];odom_history=[]
writer=cv2.VideoWriter(str(out/'front_camera.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),10.,(960,640));assert writer.isOpened()
rec=subprocess.Popen(['ffmpeg','-y','-loglevel','warning','-f','x11grab','-framerate','10','-video_size','1280x1024','-i',':138','-c:v','libx264','-preset','ultrafast','-crf','23','-pix_fmt','yuv420p',str(out/'rviz_live.mp4')],stdout=subprocess.DEVNULL,stderr=(out/'ffmpeg.log').open('w'))
def received(k):
 def f(m):
  latest[k]=m
  if k=='odom':
   q=m.pose.pose.orientation;p=m.pose.pose.position;odom_history.append(dict(elapsed=time.monotonic()-started,stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,x=p.x,y=p.y,yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))))
  if k=='native':samples.append(dict(elapsed=time.monotonic()-started,**json.loads(m.data)))
 return f
def camera(m):
 global image
 a=np.frombuffer(m.data,dtype=np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
 image=cv2.cvtColor(a,cv2.COLOR_RGB2BGR) if m.encoding=='rgb8' else a.copy()
 latest['image_stamp']=m.header.stamp.sec*10**9+m.header.stamp.nanosec

def record():
 global written
 if image is None:return
 slot=int((time.monotonic()-started)*10)
 while written<=slot:
  writer.write(image);frames.append(dict(index=written,pts=written/10,source_stamp_ns=latest.get('image_stamp')));written+=1
n.create_subscription(Odometry,'/spot1/odom',received('odom'),qos_profile_sensor_data)
n.create_subscription(String,'/wildos/native_control_status',received('native'),10)
n.create_subscription(Image,'/spot1/realsense/front/color/image_raw',camera,qos_profile_sensor_data)
def grid(m):
 grid_history.append(dict(elapsed=time.monotonic()-started,stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,frame=m.header.frame_id,width=m.info.width,height=m.info.height,resolution=m.info.resolution,origin=[m.info.origin.position.x,m.info.origin.position.y],data=list(m.data)))
def tf(m):
 tf_history.extend(dict(elapsed=time.monotonic()-started,child=t.child_frame_id,stamp_ns=t.header.stamp.sec*10**9+t.header.stamp.nanosec) for t in m.transforms if t.child_frame_id.startswith('spot1/'))
n.create_subscription(OccupancyGrid,'/local_costmap/costmap',grid,qos_profile_sensor_data)
n.create_subscription(TFMessage,'/tf',tf,qos_profile_sensor_data)
n.create_subscription(String,'/spot1/measurement_timing',lambda m:timing_history.append(dict(elapsed=time.monotonic()-started,**json.loads(m.data))),10)
n.create_timer(.1,record)
stop_pub=n.create_publisher(Twist,'/wildos/cmd_vel',1);path_pub=n.create_publisher(Path,'/wildos/planned_path',10)
def wait(f,sec):
 rclpy.spin_until_future_complete(n,f,timeout_sec=sec)
 if not f.done() or f.exception():raise RuntimeError('DDS/action response timeout/error')
 return f.result()
def parameter(owner,key):
 c=n.create_client(GetParameters,owner+'/get_parameters');assert c.wait_for_service(timeout_sec=3);v=wait(c.call_async(GetParameters.Request(names=[key])),4).values[0];n.destroy_client(c);return v

def enable(value):
 c=n.create_client(SetParameters,'/wildos_cmd_vel_to_control/set_parameters');assert c.wait_for_service(timeout_sec=3)
 p=Parameter(name='enable',value=ParameterValue(type=ParameterType.PARAMETER_BOOL,bool_value=value));r=wait(c.call_async(SetParameters.Request(parameters=[p])),4);n.destroy_client(c);assert all(v.successful for v in r.results)
report=dict(scope='Real physical car '+args.maneuver+' component; not target search or stability round',events=events,errors=errors)
try:
 t=time.monotonic()
 while time.monotonic()-t<3:rclpy.spin_once(n,timeout_sec=.1)
 assert not parameter('/wildos_cmd_vel_to_control','enable').bool_value
 assert not parameter('/wildos_graph_path_bridge','follow_path').bool_value
 assert len(n.get_publishers_info_by_topic('/wildos/native_control'))==1
 assert len(n.get_publishers_info_by_topic('/wildos/native_control_status'))==1
 native=json.loads(latest['native'].data);assert native['telemetry_ok'] and native['actuator_ok'] and native['speed']<.05
 pose=copy.deepcopy(latest['odom'].pose.pose);q=pose.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
 report['initial']=[pose.position.x,pose.position.y,yaw]
 g=ComputePathToPose.Goal();g.planner_id='CarLike';g.goal.header.frame_id='spot1/odom';g.goal.header.stamp=n.get_clock().now().to_msg();g.goal.pose=copy.deepcopy(pose);phi=(.3 if args.maneuver=='left_arc' else -.3) if args.maneuver!='reverse' else 0.;dx=10*math.sin(abs(phi)) if phi else -3.;dy=math.copysign(10*(1-math.cos(phi)),phi) if phi else 0.;g.goal.pose.position.x+=dx*math.cos(yaw)-dy*math.sin(yaw);g.goal.pose.position.y+=dx*math.sin(yaw)+dy*math.cos(yaw);g.goal.pose.orientation.x=0.;g.goal.pose.orientation.y=0.;g.goal.pose.orientation.z=math.sin((yaw+phi)/2);g.goal.pose.orientation.w=math.cos((yaw+phi)/2);report['requested_yaw']=yaw+phi
 client=ActionClient(n,ComputePathToPose,'/compute_path_to_pose');assert client.wait_for_server(timeout_sec=3);ph=wait(client.send_goal_async(g),5);assert ph.accepted;pr=wait(ph.get_result_async(),8);assert pr.status==4 and len(pr.result.path.poses)>1
 path=pr.result.path;from wildos_nav2_adapter.motion_path import motion_sections;sections=motion_sections([(p.pose.position.x,p.pose.position.y) for p in path.poses]);report['sections']=sections;assert len(sections)==1,'Component requires single motion section';path_pub.publish(path);report['path']=[[p.pose.position.x,p.pose.position.y] for p in path.poses]
 follow=ActionClient(n,FollowPath,'/follow_path');assert follow.wait_for_server(timeout_sec=3)
 fg=FollowPath.Goal();fg.path=path;fg.controller_id='FollowPath';fg.goal_checker_id='maneuver_goal_checker'
 enable(True);handle=wait(follow.send_goal_async(fg,feedback_callback=lambda m:events.append(dict(elapsed=time.monotonic()-started,feedback_distance=m.feedback.distance_to_goal,feedback_speed=m.feedback.speed))),5);assert handle.accepted
 future=handle.get_result_async();result=wait(future,60);report['action_status']=result.status
 if result.status!=4:errors.append('FollowPath did not succeed')
except Exception as exc:
 errors.append(type(exc).__name__+': '+str(exc))
finally:
 if handle is not None:
  try:wait(handle.cancel_goal_async(),2)
  except Exception as e:events.append(dict(cancel_error=str(e)))
 try:enable(False)
 except Exception as e:errors.append('Disable failed: '+str(e))
 stop_pub.publish(Twist());t=time.monotonic()
 while time.monotonic()-t<3:rclpy.spin_once(n,timeout_sec=.1)
 if 'odom' in latest:
  p=latest['odom'].pose.pose.position;report['final']=[p.x,p.y];q=latest['odom'].pose.pose.orientation;report['final_yaw']=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));report['endpoint_distance']=math.dist([p.x,p.y],report.get('path',[[p.x,p.y]])[-1])
 if 'native' in latest:report['final_native']=json.loads(latest['native'].data)
 report['passed']=not errors and report.get('action_status')==4 and report.get('endpoint_distance',99)<=.6 and report.get('final_native',{}).get('speed',99)<.05
 report['measured_translation']=math.dist(report.get('initial',[0,0])[:2],report.get('final',[0,0]));report['max_speed']=max((v.get('speed',0) for v in samples),default=0);report['samples']=samples;report['frame_index']=frames;report['elapsed']=time.monotonic()-started
 writer.release();rec.send_signal(signal.SIGINT)
 try:rec.wait(timeout=10)
 except subprocess.TimeoutExpired:rec.terminate();rec.wait(timeout=5)
 (out/'interface_witness.json').write_text(json.dumps(dict(costmaps=grid_history,timing=timing_history,tf=tf_history,odom=odom_history),indent=2));(out/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:report.get(k) for k in ('passed','errors','action_status','endpoint_distance','elapsed')}),flush=True)
 n.destroy_node();rclpy.shutdown()
