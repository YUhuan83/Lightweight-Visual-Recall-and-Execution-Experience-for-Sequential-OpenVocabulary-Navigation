"""Disable body collisions, restore sensors and actuator, retaining the map."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
sys.path.insert(0,'/home/ly')
from session_tools_20261002 import ROOT, WS, runtime_env
if '--runtime-child' not in sys.argv:
    raise SystemExit(subprocess.run([str(WS/'.venv/bin/python'),__file__,'--runtime-child'],env=runtime_env()).returncode)

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType
from rcl_interfaces.srv import SetParameters
from set_beamng_body_collision import set_body_collision
out=ROOT/'body_collision_full'
out.mkdir(exist_ok=True)
rclpy.init()
node=Node('prepare_body_collision_full')
try:
    for remote,key in [('/wildos_graph_path_bridge','follow_path'),('/wildos_cmd_vel_to_control','enable')]:
        c=node.create_client(SetParameters,remote+'/set_parameters')
        if not c.wait_for_service(timeout_sec=5): raise RuntimeError('Missing '+remote)
        f=c.call_async(SetParameters.Request(parameters=[Parameter(name=key,value=ParameterValue(type=ParameterType.PARAMETER_BOOL,bool_value=False))]))
        rclpy.spin_until_future_complete(node,f,timeout_sec=5)
        if not f.done() or not all(v.successful for v in f.result().results): raise RuntimeError('Disable failed '+remote)
finally:
    node.destroy_node()
    rclpy.shutdown()

def alive(pid):
    try: return Path('/proc',str(pid),'stat').read_text().split()[2]!='Z'
    except FileNotFoundError: return False

def stop(pid):
    if not alive(pid): return
    os.kill(pid,signal.SIGINT)
    deadline=time.monotonic()+12
    while alive(pid) and time.monotonic()<deadline: time.sleep(.1)
    if alive(pid): raise RuntimeError('Process did not stop: '+str(pid))

native_pid=int((ROOT/'native_control.pid').read_text())
stop(native_pid)
bridges=[]
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():continue
    try: args=proc.joinpath('cmdline').read_bytes().decode().split('\0')[:-1]
    except (OSError,UnicodeError):continue
    if any(a.endswith('/lib/beamng_ros2/beamng_bridge') or a.endswith('/attach_current_town_bridge.py') for a in args):
        bridges.append(int(proc.name))
for pid in bridges: stop(pid)
print('Stopped sensor bridge and native actuator; map retained',flush=True)
result=set_body_collision(host='127.0.0.1',port=25252,vehicle_name='ego',disabled=True,
    extension=Path('/home/ly/wildosBodyCollision.lua'),output=out/'profile')
print(json.dumps({'applied':result['applied'],'counts':result['after']['liveCounts'],
    'sensors_before_recreate':{'camera':result['after']['cameras'],'lidar':result['after']['lidars']}}),flush=True)
restored=subprocess.run([str(WS/'.venv/bin/python'),'/home/ly/start_body_collision_runtime.py'],
    env=runtime_env(),timeout=30)
if restored.returncode: raise RuntimeError('Collision-disabled runtime recovery failed')
print('Body collision disabled; sensors and actuator restored without map reload',flush=True)

