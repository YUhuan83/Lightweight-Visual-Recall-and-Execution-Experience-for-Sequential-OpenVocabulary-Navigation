"""Apply the requested profile with the native actuator disconnected briefly."""
from pathlib import Path
import json
import os
import signal
import subprocess
import sys
import time

sys.path.insert(0, '/home/ly')
from session_tools_20261002 import ROOT, WS, runtime_env
if '--runtime-child' not in sys.argv:
    raise SystemExit(subprocess.run([str(WS/'.venv/bin/python'), __file__,
                                    *sys.argv[1:], '--runtime-child'], env=runtime_env()).returncode)
from set_beamng_body_collision import set_body_collision
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters

out=ROOT/'body_collision';out.mkdir(exist_ok=True)
rclpy.init();node=Node('apply_body_collision_profile')
try:
    for remote,key in [('/wildos_graph_path_bridge','follow_path'),('/wildos_cmd_vel_to_control','enable')]:
        client=node.create_client(SetParameters, remote+'/set_parameters')
        if not client.wait_for_service(timeout_sec=5):raise RuntimeError('Missing '+remote)
        future=client.call_async(SetParameters.Request(parameters=[Parameter(name=key,
            value=ParameterValue(type=ParameterType.PARAMETER_BOOL,bool_value=False))]))
        rclpy.spin_until_future_complete(node,future,timeout_sec=5)
        if not future.done() or not all(v.successful for v in future.result().results):
            raise RuntimeError('Execution disable failed: '+remote)
finally:
    node.destroy_node();rclpy.shutdown()

pid=int((ROOT/'native_control.pid').read_text());proc=Path('/proc',str(pid))
args=proc.joinpath('cmdline').read_bytes().decode().split('\0')[:-1]
environment=dict(v.split('=',1) for v in proc.joinpath('environ').read_bytes().decode().split('\0') if '=' in v)
cwd=str(proc.joinpath('cwd').resolve())
os.kill(pid,signal.SIGINT)
deadline=time.monotonic()+15
while proc.exists() and time.monotonic()<deadline:
    if proc.joinpath('stat').read_text().split()[2]=='Z':break
    time.sleep(.1)
if proc.exists() and proc.joinpath('stat').read_text().split()[2]!='Z':
    raise RuntimeError('Native actuator did not stop; no profile change attempted')
try:
    result=set_body_collision(host='127.0.0.1',port=25252,vehicle_name='ego',
        disabled='--restore' not in sys.argv,extension=Path('/home/ly/wildosBodyCollision.lua'),output=out)
    print(json.dumps({'applied':result['applied'],'map_reload_count':result['map_reload_count'],
                      'after':{k:v for k,v in result['after'].items() if k not in ('nodeMasks',)}}),flush=True)
finally:
    with (out/'native_control.log').open('ab') as log:
        restored=subprocess.Popen(args,cwd=cwd,env=environment,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    (ROOT/'native_control.pid').write_text(str(restored.pid))
    time.sleep(1)
    if restored.poll() is not None:raise RuntimeError('Native actuator failed to reconnect; inspect native_control.log')
    print('Native actuator restored:',restored.pid,flush=True)
