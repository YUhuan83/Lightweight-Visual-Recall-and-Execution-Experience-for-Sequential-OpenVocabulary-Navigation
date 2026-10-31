"""Finish restoring the runtime after the verified collision profile."""
import subprocess
from pathlib import Path
import sys
import time
sys.path.insert(0,'/home/ly')
from session_tools_20261002 import ROOT,WS,runtime_env
env=runtime_env()
for pid_file in (ROOT/'body_collision_full/bridge.pid', ROOT/'native_control.pid'):
    if not pid_file.exists(): continue
    status=Path('/proc',pid_file.read_text().strip(),'stat')
    if status.exists() and status.read_text().split()[2]!='Z':
        raise RuntimeError('Runtime already active: '+str(pid_file))
from beamngpy import BeamNGpy
b=BeamNGpy('127.0.0.1',25252,socket_timeout=8).open(launch=False)
b.control.resume()
b.disconnect()
out=ROOT/'body_collision_full'
with (out/'bridge.log').open('a') as log:
    p=subprocess.Popen([str(WS/'.venv/bin/python'),'/home/ly/attach_current_town_bridge.py'],cwd=WS,
        env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
(out/'bridge.pid').write_text(str(p.pid))
time.sleep(6)
if p.poll() is not None:raise RuntimeError((out/'bridge.log').read_text()[-2500:])
import json
profile=json.loads((out/'profile/application.json').read_text())
args=[str(WS/'.venv/bin/python'),str(WS/'install/wildos_nav2_adapter/lib/wildos_nav2_adapter/native_vehicle_control'),
    '--ros-args','-p','overspeed_limit:=0.0','-p','command_timeout:=0.8',
    '-p','vehicle_port:='+str(profile['known_vehicle_port'])]
with (out/'native_control.log').open('a') as log:
    native=subprocess.Popen(args,cwd=WS,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
(ROOT/'native_control.pid').write_text(str(native.pid))
time.sleep(2)
if native.poll() is not None: raise RuntimeError((out/'native_control.log').read_text()[-2000:])
print('Collision-disabled runtime restored; map reloads=0',flush=True)

