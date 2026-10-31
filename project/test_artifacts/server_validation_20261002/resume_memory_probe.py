"""Continue memory return at held pose; no map reload, teleport or vision reset."""
import json
import math
from pathlib import Path
import subprocess
import sys
import time
sys.path.insert(0,'/home/ly')
from session_tools_20261002 import ROOT,WS,runtime_env
if '--runtime-child' not in sys.argv:
    raise SystemExit(subprocess.run([str(WS/'.venv/bin/python'),__file__,'--runtime-child'],env=runtime_env()).returncode)
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType
from rcl_interfaces.srv import SetParameters
from beamngpy import BeamNGpy

out=ROOT/'held_memory_return_geometry';out.mkdir(exist_ok=True)
vision_pid=int(ROOT.joinpath('vision.pid').read_text())
b=BeamNGpy('127.0.0.1',25252,socket_timeout=5);b.open(launch=False)
rclpy.init();n=Node('held_memory_return_probe');received={};events=[];native_samples=[]
def cb(key,m):
    if key=='odom':received[key]=m;return
    value=json.loads(m.data);received[key]=value
    if key=='native':native_samples.append(value);return
    if not events or events[-1].get('data')!=value:
        e={'wall_time':time.time(),'kind':key,'data':value};events.append(e);print(json.dumps(e),flush=True)
for key,topic in [('native','/wildos/native_control_status'),('status','/wildos/plan_status'),('selection','/wildos/memory_selection')]:
    n.create_subscription(String,topic,lambda m,key=key:cb(key,m),10)
n.create_subscription(Odometry,'/spot1/odom',lambda m:cb('odom',m),qos_profile_sensor_data)
query=n.create_publisher(String,'/spot1/object_search/query',10)
def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
def set_bool(remote,key,value):
    c=n.create_client(SetParameters,remote+'/set_parameters')
    if not c.wait_for_service(timeout_sec=3):raise RuntimeError('Missing '+remote)
    f=c.call_async(SetParameters.Request(parameters=[Parameter(name=key,value=ParameterValue(type=ParameterType.PARAMETER_BOOL,bool_value=value))]))
    rclpy.spin_until_future_complete(n,f,timeout_sec=3)
    if not f.done() or not all(v.successful for v in f.result().results):raise RuntimeError('Parameter failed '+key)
report={'scope':'held-pose memory-return regression; first house goal and 20 m departure were not completed',
        'full_sequence_acceptance':False,'map_reloads':0,'teleports':0,'vision_pid':vision_pid,'reached_memory_goal':False}
try:
    set_bool('/wildos_graph_path_bridge','follow_path',False);set_bool('/wildos_cmd_vel_to_control','enable',False)
    b.control.resume();spin(2)
    report['start_position']=received['native']['position']
    query.publish(String(data='gas station'));spin(4)
    set_bool('/wildos_cmd_vel_to_control','enable',True);set_bool('/wildos_graph_path_bridge','follow_path',True)
    until=time.monotonic()+100
    while time.monotonic()<until:
        spin(.1)
        if not Path('/proc',str(vision_pid)).exists():raise RuntimeError('Vision process exited')
        selection=received.get('selection',{});odom=received.get('odom')
        if selection.get('status')=='memory_candidate_selected' and odom is not None:
            p=odom.pose.pose.position;goal=selection['goal']
            error=math.hypot(p.x-goal[0],p.y-goal[1])
            report['final_navigation_error']=error
            if error<1.0 and received['native']['speed']<.2:
                report['reached_memory_goal']=True;break
    report['final_position']=received['native']['position'];report['vision_pid_retained']=Path('/proc',str(vision_pid)).exists()
except BaseException as exc:report['error']=repr(exc)
finally:
    try:
        set_bool('/wildos_graph_path_bridge','follow_path',False);set_bool('/wildos_cmd_vel_to_control','enable',False)
        spin(1)
    finally:
        b.control.pause();b.disconnect()
        report['maximum_native_displacement']=max((math.dist(s['position'][:2],report['start_position'][:2]) for s in native_samples),default=0.) if 'start_position' in report else None
        report['navigation_execution_verified']=bool(report['maximum_native_displacement'] and report['maximum_native_displacement']>.5 and any(e.get('data',{}).get('status') in ('maneuver_done','segment_done') for e in events))
        report['paused']=True;report['events']=events
        out.joinpath('result.json').write_text(json.dumps(report,indent=2))
        out.joinpath('native_samples.jsonl').write_text(''.join(json.dumps(s)+'\n' for s in native_samples))
        n.destroy_node();rclpy.shutdown()
print(json.dumps({k:v for k,v in report.items() if k!='events'},indent=2),flush=True)
