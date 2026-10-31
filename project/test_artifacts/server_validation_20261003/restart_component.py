"""Restart one reviewed component; retain the map and sensor bridge."""
from pathlib import Path
import json
import os
import signal
import subprocess
import sys
import time

ROOT = Path('/home/ly/town_memory_validation_20261003')
name = sys.argv[1]
if name not in ('graph_bridge', 'triangulation', 'speed_control', 'vision', 'sensor_bridge',
                'mapper', 'graphnav_planner', 'goal_standoff', 'visualization', 'official_adapter',
                'planner', 'controller', 'lifecycle'):
    raise ValueError('Unreviewed component')
rows = {r['name']:r for r in json.loads((ROOT/'restart_snapshot.json').read_text())}
row = rows[name]
pid = int((ROOT/(name+'.pid')).read_text())
cmd = Path('/proc')/str(pid)/'cmdline'
if cmd.exists():
    identity = row['args'][0] if name in ('mapper', 'graphnav_planner', 'planner', 'controller', 'lifecycle') else row['args'][1]
    assert identity.encode() in cmd.read_bytes(), 'Process identity changed'
    os.kill(pid,signal.SIGINT)
    deadline = time.monotonic()+15
    while cmd.exists() and time.monotonic()<deadline:
        status = (cmd.parent/'status').read_text()
        if 'State:\tZ' in status:
            break
        time.sleep(.2)
    else:
        if cmd.exists():
            raise RuntimeError('Component did not shut down')
args = list(row['args'])
if name=='triangulation':
    args[args.index('--config')+1] = str(ROOT/'triangulation_town.yaml')
elif name=='vision':
    args[args.index('--config')+1] = str(ROOT/'vision_town.yaml')
env = dict(row['environment'])
env.update(ROS_DOMAIN_ID='40',ROS_LOCALHOST_ONLY='1',ROS2CLI_NO_DAEMON='1')
if name == 'sensor_bridge':
    check = '''
from beamngpy import BeamNGpy
import json
b=BeamNGpy('127.0.0.1',25252,socket_timeout=6).open(launch=False)
try:
 data=json.loads(b.control.queue_lua_command("return jsonEncode({mission=getMissionFilename(),cameras=extensions.tech_sensors.getActiveCameraSensors(),lidars=extensions.tech_sensors.getActiveLidarSensors()})",response=True))
 assert data['mission']=='/levels/east_coast_usa/main.level.json', 'Map identity changed'
 assert not data['cameras'] and not data['lidars'], 'Owned sensor cleanup incomplete; do not duplicate sensors'
 print(json.dumps(data))
finally: b.disconnect()
'''
    subprocess.run(['/home/ly/beamng_ros2_0394_ws/.venv/bin/python','-c',check],env=env,check=True,timeout=15)
with (ROOT/(name+'.log')).open('a') as log:
    process = subprocess.Popen(args,cwd=row['cwd'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
(ROOT/(name+'.pid')).write_text(str(process.pid))
time.sleep(1)
if process.poll() is not None:
    raise RuntimeError('Restart failed: '+name)
print(json.dumps({'name':name,'previous_pid':pid,'pid':process.pid}))
