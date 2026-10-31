"""Inspect exact active configs/logs; refresh paused viewer with controls disabled."""
import json, os, sys, subprocess, time
from pathlib import Path
sys.path.insert(0,'/home/ly')
from session_tools_20261002 import ROOT,WS,runtime_env
if '--runtime-child' not in sys.argv:
    raise SystemExit(subprocess.run([str(WS/'.venv/bin/python'),__file__,'--runtime-child'],env=runtime_env()).returncode)
import rclpy, cv2, numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters,ListParameters
from sensor_msgs.msg import Image
from std_msgs.msg import String
from beamngpy import BeamNGpy
out=ROOT/'mechanism_audit'
rclpy.init(); n=Node('mechanism_visual_refresh'); result={'params':{},'images':{},'log_paths':{},'feedback':[]}
def call(remote,names=None):
    c=n.create_client(ListParameters if names is None else GetParameters,remote+('/list_parameters' if names is None else '/get_parameters'))
    if not c.wait_for_service(timeout_sec=3): raise RuntimeError('missing '+remote)
    f=c.call_async(ListParameters.Request() if names is None else GetParameters.Request(names=names))
    rclpy.spin_until_future_complete(n,f,timeout_sec=4);n.destroy_client(c)
    if not f.done():raise RuntimeError('timeout '+remote)
    return f.result()
for remote in ('/spot1/mapper_node','/beamng_vehicle_goal_standoff','/spot1/graphnav_path_follower'):
    names=call(remote).result.names
    values=call(remote,names).values
    attrs={1:'bool_value',2:'integer_value',3:'double_value',4:'string_value',5:'byte_array_value',6:'bool_array_value',7:'integer_array_value',8:'double_array_value',9:'string_array_value'}
    result['params'][remote]={k:(list(getattr(v,attrs[v.type])) if v.type>=5 else getattr(v,attrs[v.type],None)) for k,v in zip(names,values) if v.type}
for name in ('vision','triangulation','standoff','mapper','graphnav_planner','graph_bridge','path_follower'):
    p=ROOT/(name+'.pid')
    if not p.exists():continue
    pid=p.read_text().strip()
    try:
        log=Path(os.readlink('/proc/'+pid+'/fd/1'))
        result['log_paths'][name]=str(log)
        selected=[]
        with log.open(errors='replace') as f:
            for line in f:
                if '[1790940' in line and any(k in line for k in ('Received callback','LOCK','Not enough','too close','Triangulated','LiDAR points','stable object','Stored visual','matching visual','changed to','Stored','Traceback')):
                    selected.append(line)
        (out/(name+'_trial_excerpt.log')).write_text(''.join(selected))
        result[name+'_log_lines']=len(selected)
    except OSError as e:result[name+'_log_error']=str(e)
from ament_index_python.packages import get_package_share_directory
for name in ('beamng_wildos_town_full_20260930.yaml','triangulation_town_full_20260930.yaml'):
    p=Path(get_package_share_directory('visual_navigation'))/'configs'/name
    (out/name).write_bytes(p.read_bytes())
    result.setdefault('config_paths',{})[name]=str(p)
def image_receive(key,m):
    a=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
    if m.encoding=='rgb8':a=cv2.cvtColor(a,cv2.COLOR_RGB2BGR)
    cv2.imwrite(str(out/(key+'.png')),a)
    result['images'][key]={'count':result['images'].get(key,{}).get('count',0)+1,'width':m.width,'height':m.height,'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9}
for key,topic in [('model','/spot1/model_visualization'),('frontier_dashboard','/wildos/viz/frontier_traversability')]:
    n.create_subscription(Image,topic,lambda m,key=key:image_receive(key,m),qos_profile_sensor_data)
enabled=call('/wildos_cmd_vel_to_control',['enable']).values[0].bool_value
following=call('/wildos_graph_path_bridge',['follow_path']).values[0].bool_value
if enabled or following:raise RuntimeError('Controls must remain disabled')
b=BeamNGpy('127.0.0.1',25252,socket_timeout=8);b.open(launch=False)
video=None
try:
    b.control.resume()
    video=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','error','-y','-f','x11grab','-framerate','5','-video_size','1280x1024','-i',':99','-t','18','-c:v','libx264','-preset','ultrafast','-crf','24','-pix_fmt','yuv420p',str(out/'rviz_frontier_preview.mp4')])
    until=time.monotonic()+20
    while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.05)
    subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-f','x11grab','-video_size','1280x1024','-i',':99','-frames:v','1','-threads','1',str(out/'rviz_frontier.png')],check=True,timeout=10)
finally:
    b.control.pause();b.disconnect()
    if video is not None:video.wait(timeout=15)
    n.destroy_node();rclpy.shutdown()
    (out/'visual_refresh.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))

