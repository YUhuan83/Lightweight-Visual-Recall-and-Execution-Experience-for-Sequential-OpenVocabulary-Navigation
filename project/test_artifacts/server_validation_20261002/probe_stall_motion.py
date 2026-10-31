"""Bounded manual direction comparison, using one actuator and the loaded map."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
sys.path.insert(0,'/home/ly')
from session_tools_20261002 import ROOT,WS,runtime_env
if '--runtime-child' not in sys.argv:
    raise SystemExit(subprocess.run([str(WS/'.venv/bin/python'),__file__,*sys.argv[1:],'--runtime-child'],env=runtime_env()).returncode)
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType
from rcl_interfaces.srv import SetParameters
from beamngpy import BeamNGpy
from beamngpy.sensors import Electrics
rclpy.init();n=Node('station_stall_motion_probe')
for remote,key in [('/wildos_graph_path_bridge','follow_path'),('/wildos_cmd_vel_to_control','enable')]:
    client=n.create_client(SetParameters,remote+'/set_parameters')
    if not client.wait_for_service(timeout_sec=5):raise RuntimeError('Missing '+remote)
    future=client.call_async(SetParameters.Request(parameters=[Parameter(name=key,value=ParameterValue(type=ParameterType.PARAMETER_BOOL,bool_value=False))]))
    rclpy.spin_until_future_complete(n,future,timeout_sec=10)
    if not future.done() or not all(v.successful for v in future.result().results):raise RuntimeError('Cannot disable '+remote)
    n.destroy_client(client)
n.destroy_node();rclpy.shutdown()
b=BeamNGpy('127.0.0.1',25252,socket_timeout=8);b.open(launch=False);b.control.pause()
pid=int((ROOT/'native_control.pid').read_text());proc=Path('/proc',str(pid))
args=proc.joinpath('cmdline').read_bytes().decode().split('\0')[:-1]
env=dict(v.split('=',1) for v in proc.joinpath('environ').read_bytes().decode().split('\0') if '=' in v)
os.kill(pid,signal.SIGINT);end=time.monotonic()+15
while proc.exists() and time.monotonic()<end:
    if proc.joinpath('stat').read_text().split()[2]=='Z':break
    time.sleep(.1)
if proc.exists() and proc.joinpath('stat').read_text().split()[2]!='Z':raise RuntimeError('Native actuator did not stop')
v=None;report={'map_reload_count':0,'teleports':0,'samples':[]}
reproduce='reverse_reproduce' in sys.argv
try:
    v=b.vehicles.get_current()['ego'];v.connect(b);v.sensors.attach('stall_motion_electrics',Electrics())
    v.ai.set_mode('disabled');v.set_shift_mode('realistic_automatic');v.sensors.poll()
    origin=list(v.state['pos']);report['initial_position']=origin
    def sample(label):
        v.sensors.poll();state=dict(v.state);electric=dict(v.sensors['stall_motion_electrics'])
        value={'label':label,'state':state,'electrics':{k:electric.get(k) for k in ('throttle','brake','parkingbrake','rpm','gear','wheelspeed','clutchRatio1')},
               'displacement_m':math.dist(state['pos'][:2],origin[:2])}
        report['samples'].append(value);return value
    cases=[('reverse_reproduce',-1,0.)] if reproduce else [('reverse_original_steering',-1,-.1708788),('reverse_straight',-1,0.),('forward_straight',2,0.)]
    stationary_samples=0
    for label,gear,steering in cases:
        v.control(throttle=0,brake=1,parkingbrake=0,clutch=0,steering=steering,gear=gear)
        b.control.step(30,wait=True);sample(label+'_shift')
        for i in range(45 if reproduce else 15):
            v.control(throttle=.12,brake=0,parkingbrake=0,clutch=0,steering=steering,gear=gear)
            b.control.step(6,wait=True);current=sample(label)
            speed=math.hypot(*current['state']['vel'][:2])
            if reproduce:
                stationary_samples=stationary_samples+1 if speed<.015 and current['displacement_m']>.5 else 0
                if stationary_samples>=6:
                    report['stall_reproduced']=True
                    report['contact_bbox']=v.get_bbox()
                    break
            if current['displacement_m']>(2.5 if reproduce else 1.):
                report['distance_guard_triggered']=True;break
        v.control(throttle=0,brake=1,parkingbrake=0,steering=0,clutch=0,gear=gear)
        b.control.step(20,wait=True)
        if report.get('distance_guard_triggered'):break
    sample('stopped');report['simulation_paused']=True
except BaseException as exc:
    report['error']=repr(exc)
finally:
    if v is not None:
        try:
            v.control(throttle=0,brake=1,parkingbrake=1,steering=0,clutch=0,gear=0)
            b.control.step(10,wait=True)
            v.sensors.detach('stall_motion_electrics');v.disconnect()
        except Exception as exc:report['stop_error']=repr(exc)
        finally:
            try:b.control.pause()
            except Exception as exc:report['pause_error']=repr(exc)
    b.disconnect()
    (ROOT/('stall_reverse_reproduction.json' if reproduce else 'stall_motion_comparison.json')).write_text(json.dumps(report,indent=2))
    with (ROOT/'navigation_repair/native_control.log').open('ab') as log:
        p=subprocess.Popen(args,cwd=WS,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    (ROOT/'native_control.pid').write_text(str(p.pid))
    print(json.dumps({'native_restarted':p.pid,'samples':[{'label':s['label'],'displacement_m':s['displacement_m'],'speed':math.hypot(*s['state']['vel'][:2]),'gear':s['electrics']['gear'],'rpm':s['electrics']['rpm']} for s in report['samples']]}),flush=True)
