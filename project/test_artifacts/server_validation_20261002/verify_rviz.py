"""Inspect live visualization topics and TF while the formal test is running."""
import json
import math
from pathlib import Path
import subprocess
import sys
import time
sys.path.insert(0,'/home/ly')
from session_tools_20261002 import ROOT,WS,ros_environment,runtime_env
if '--runtime-child' not in sys.argv:
    raise SystemExit(subprocess.run([str(WS/'.venv/bin/python'),__file__,'--runtime-child'],env=runtime_env()).returncode)
ros_environment()
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry,OccupancyGrid,Path as Route
from sensor_msgs.msg import PointCloud2,Image
from geometry_msgs.msg import PolygonStamped
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from tf2_ros import Buffer,TransformListener
from rcl_interfaces.srv import GetParameters

out=ROOT/'geometry_rviz_verification';out.mkdir(exist_ok=True)
ffmpeg=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','warning','-y','-f','x11grab',
    '-filter_threads','1','-video_size','1280x1024','-framerate','5','-i',':99','-t','180','-c:v','libx264',
    '-threads','1','-preset','ultrafast','-pix_fmt','yuv420p',str(out/'rviz_live.mp4')],
    stdout=out.joinpath('recording.log').open('ab'),stderr=subprocess.STDOUT,start_new_session=True)
out.joinpath('recording.pid').write_text(str(ffmpeg.pid))
rclpy.init();n=Node('beamng_rviz_verification');received={};counts={}
def cb(key,m):received[key]=m;counts[key]=counts.get(key,0)+1
for key,kind,topic in [('odom',Odometry,'/spot1/odom'),('cloud',PointCloud2,'/spot1/ouster/front/points_filtered'),
    ('camera',Image,'/spot1/realsense/front/color/image_raw'),('local',OccupancyGrid,'/wildos/viz/local_costmap'),
    ('global',OccupancyGrid,'/wildos/viz/global_costmap'),('footprint',PolygonStamped,'/wildos/viz/footprint'),
    ('path',Route,'/wildos/viz/planned_path'),('trail',Route,'/wildos/odom_trail'),
    ('markers',MarkerArray,'/wildos/debug_markers'),('native',String,'/wildos/native_control_status')]:
    n.create_subscription(kind,topic,lambda m,key=key:cb(key,m),qos_profile_sensor_data)
buffer=Buffer();listener=TransformListener(buffer,n)
until=time.monotonic()+15
while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.05)
report={'counts':counts,'domain':40,'display':':99','rviz_pid':int(ROOT.joinpath('rviz.pid').read_text())}
if 'odom' in received:
    odom=received['odom'];p=odom.pose.pose.position;q=odom.pose.pose.orientation
    heading=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    report['odom']={'position':[p.x,p.y,p.z],'heading':heading,'child_frame':odom.child_frame_id}
    report['plane_heights']={k:received[k].info.origin.position.z for k in ('local','global') if k in received}
    try:
        transform=buffer.lookup_transform('spot1/base_link','spot1/ouster/front/os_lidar',rclpy.time.Time())
        t=transform.transform.translation;report['base_to_cloud_origin']=[t.x,t.y,t.z]
    except Exception as exc:report['tf_error']=str(exc)
if 'native' in received:report['native']=json.loads(received['native'].data)
report['parameters']={}
for remote,names in [('/beamng_wildos_adapter',['base_offset']),('/wildos_cmd_vel_to_control',['wheelbase','max_steer_angle']),
                      ('/planner_server',['CarLike.max_planning_time','CarLike.minimum_turning_radius']),
                      ('/local_costmap/local_costmap',['robot_base_frame','footprint','obstacle_layer.cloud.sensor_frame'])]:
    client=n.create_client(GetParameters,remote+'/get_parameters')
    if not client.wait_for_service(timeout_sec=3):raise RuntimeError('Missing '+remote)
    f=client.call_async(GetParameters.Request(names=names));rclpy.spin_until_future_complete(n,f,timeout_sec=3)
    if not f.done():raise RuntimeError('Parameter timeout '+remote)
    result={}
    for key,value in zip(names,f.result().values):
        result[key]=value.double_array_value.tolist() if value.type==8 else value.double_value if value.type==3 else value.string_value
    report['parameters'][remote]=result
report['required_visual_topics_present']=all(counts.get(k,0)>0 for k in ('odom','cloud','camera','local','global','footprint','markers','trail'))
out.joinpath('verification.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2),flush=True)
capture=subprocess.run(['ffmpeg','-hide_banner','-loglevel','warning','-y','-filter_threads','1','-f','x11grab','-video_size','1280x1024','-i',':99',
    '-frames:v','1','-threads','1',str(out/'rviz_live.png')],capture_output=True,text=True,timeout=10)
if capture.returncode:raise RuntimeError('RViz capture failed: '+capture.stderr)
n.destroy_node();rclpy.shutdown()
