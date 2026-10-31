"""Restore the reviewed Town runtime after build, retaining the loaded map."""
from pathlib import Path
import json
import os
import subprocess
import time

ROOT = Path('/home/ly/town_memory_validation_20261003')
WS = Path('/home/ly/beamng_ros2_0394_ws')
OLD = Path('/home/ly/full_town_memory_20261002')
rows = {r['name']: r for r in json.loads((ROOT/'restart_snapshot.json').read_text())}
assert json.loads((ROOT/'build_result.json').read_text())['returncode'] == 0
tests = json.loads((ROOT/'cpp_test_result.json').read_text())
assert all(t['returncode'] == 0 and t['failures'] == 0 and t['errors'] == 0 for t in tests)

env = dict(os.environ, **json.loads((OLD/'runtime_env.json').read_text()))
env.update(ROS_DOMAIN_ID='40', ROS_LOCALHOST_ONLY='1', ROS2CLI_NO_DAEMON='1')

cleanup = r'''
from beamngpy import BeamNGpy
from pathlib import Path
import json
b = BeamNGpy('127.0.0.1',25252,socket_timeout=6).open(launch=False)
try:
    b.control.pause()
    result = json.loads(b.control.queue_lua_command("""
local v=scenetree.findObject('ego')
assert(v and v:getID()==78203,'Vehicle identity changed')
assert(getMissionFilename()=='/levels/east_coast_usa/main.level.json','Map identity changed')
local t=extensions.tech_sensors
local c=t.getActiveCameraSensors()
local l=t.getActiveLidarSensors()
assert(#c==1 and c[1]==13 and #l==1 and l[1]==14,'Unexpected remaining sensor ownership')
t.removeSensor(13)
t.removeSensor(14)
return jsonEncode({removed={13,14},cameras=t.getActiveCameraSensors(),lidars=t.getActiveLidarSensors(),mission=getMissionFilename()})
""",response=True))
    b.vehicles.teleport('ego',(734.294,-11.946,52.154),(0.,0.,.7658087977066911,.6430683364581344),reset=True)
    Path('/home/ly/town_memory_validation_20261003/sensor_cleanup.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result),flush=True)
finally:
    b.disconnect()
'''
subprocess.run([str(WS/'.venv/bin/python'), '-c', cleanup], env=env, check=True, timeout=20)

import yaml
vision = yaml.safe_load((ROOT/'original_beamng_wildos_town_full_20260930.yaml').read_text())
vision['object_search_config']['text_queries'] = ['gas station']
vision['visual_memory_map_id'] = 'town-session-20261003'
vision['visual_memory_path'] = vision['visual_memory_map_path'] = ':memory:'
vision['visual_memory_map_save_images'] = False
(ROOT/'vision_town.yaml').write_text(yaml.safe_dump(vision,sort_keys=False))
triang = yaml.safe_load((ROOT/'original_triangulation_town_full_20260930.yaml').read_text())
triang.update(max_views=12,sparse_lidar_points=8,observation_timeout_sec=5.0)
(ROOT/'triangulation_town.yaml').write_text(yaml.safe_dump(triang,sort_keys=False))

started = {}
def start(name):
    row = rows[name]
    args = list(row['args'])
    environment = dict(row['environment'])
    environment.update(ROS_DOMAIN_ID='40',ROS_LOCALHOST_ONLY='1',ROS2CLI_NO_DAEMON='1',
                       WILDOS_TEST_SESSION_DIR=str(ROOT))
    if name == 'vision':
        args[args.index('--config')+1] = str(ROOT/'vision_town.yaml')
    elif name == 'triangulation':
        args[args.index('--config')+1] = str(ROOT/'triangulation_town.yaml')
    elif name == 'graph_bridge':
        assert 'follow_path:=false' in args
    elif name == 'speed_control':
        assert 'enable:=false' in args and 'max_speed:=0.0' in args and 'overspeed_limit:=0.0' in args
    with (ROOT/(name+'.log')).open('w') as log:
        p = subprocess.Popen(args,cwd=row['cwd'],env=environment,stdout=log,
                             stderr=subprocess.STDOUT,start_new_session=True)
    (ROOT/(name+'.pid')).write_text(str(p.pid))
    started[name] = dict(pid=p.pid,args=args)
    time.sleep(.35)
    if p.poll() is not None:
        raise RuntimeError(name+' exited: '+(ROOT/(name+'.log')).read_text(errors='replace')[-1800:])
    print(json.dumps(dict(started=name,pid=p.pid)),flush=True)

for name in ('sensor_bridge','native_control','official_adapter','cloud_filter',
             'pose_graph_publisher','mapper','graphnav_planner','goal_standoff',
             'graph_bridge','speed_control','planner','controller','lifecycle',
             'triangulation','vision','visualization'):
    start(name)
(ROOT/'runtime_started.json').write_text(json.dumps(started,indent=2))
print('RUNTIME_STARTED; loaded Town map retained',flush=True)
