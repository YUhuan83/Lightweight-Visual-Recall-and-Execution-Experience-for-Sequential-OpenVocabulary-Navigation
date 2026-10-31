import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

ws = Path('/home/ly/beamng_ros2_0394_ws')
stage = Path('/home/ly/navigation_validation_20260930_stage')
backup = Path('/home/ly/navigation_validation_20260930_backup')
logs = Path('/home/ly/navigation_validation_20260930_logs')
logs.mkdir(exist_ok=True)
if backup.exists():
    raise RuntimeError('Backup exists: inspect previous deployment before rerunning')
backup.mkdir()
names = {'beamng_wildos_adapter', 'cloud_filter', 'planner_node', 'cmd_vel_to_control'}
processes = {}
for p in Path('/proc').iterdir():
    if not p.name.isdigit():
        continue
    try:
        argv = p.joinpath('cmdline').read_bytes().decode().split('\0')[:-1]
    except (OSError, UnicodeError):
        continue
    for name in names:
        if any(a == str(ws / 'install' / ('graphnav_planner' if name == 'planner_node' else 'beamng_wildos_adapter' if name == 'beamng_wildos_adapter' else 'wildos_nav2_adapter') / 'lib' / ('graphnav_planner' if name == 'planner_node' else 'beamng_wildos_adapter' if name == 'beamng_wildos_adapter' else 'wildos_nav2_adapter') / name) for a in argv):
            if name in processes:
                raise RuntimeError('Duplicate process: ' + name)
            processes[name] = {'pid': int(p.name), 'argv': argv}
if set(processes) != names:
    raise RuntimeError('Missing expected processes: ' + str(names - set(processes)))
backup.joinpath('processes.json').write_text(json.dumps(processes, indent=2))

def launch(argv, name):
    with logs.joinpath(name + '.log').open('ab') as log:
        p = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    logs.joinpath(name + '.pid').write_text(str(p.pid))
    print('started', name, p.pid, flush=True)
    return p

def stop(name):
    pid = processes[name]['pid']
    os.kill(pid, signal.SIGINT)
    deadline = time.monotonic() + 15
    while Path('/proc', str(pid)).exists() and time.monotonic() < deadline:
        time.sleep(.2)
    if Path('/proc', str(pid)).exists():
        raise RuntimeError('Node did not exit: ' + name)
    print('stopped', name, pid, flush=True)

# The outgoing control node publishes brakes during shutdown.
stop('cmd_vel_to_control')
control = list(processes['cmd_vel_to_control']['argv'])
control = [a.replace('enable:=true', 'enable:=false').replace('max_steering:=0.4', 'max_steering:=0.6') for a in control]
launch(control, 'control_hold')
time.sleep(1)
for name in ('planner_node', 'cloud_filter', 'beamng_wildos_adapter'):
    stop(name)
copies = [(stage / 'official_adapter', ws / 'src/beamng_wildos_adapter'),
          (stage / 'nav2_adapter', ws / 'src/wildos_nav2_adapter'),
          (stage / 'wildos_port/repro_check/src/nebula2-wildos/graphnav_planner', ws / 'src/nebula2-wildos/graphnav_planner')]
for src, dst in copies:
    shutil.copytree(dst, backup / dst.name, symlinks=True)
    shutil.copytree(src, dst, dirs_exist_ok=True)
print('backup and source update complete', flush=True)
subprocess.run(['colcon', 'build', '--packages-select', 'wildos_nav2_adapter', 'beamng_wildos_adapter', 'graphnav_planner', '--symlink-install', '--executor', 'sequential'], cwd=ws, check=True)
for name in ('beamng_wildos_adapter', 'cloud_filter', 'planner_node'):
    argv = processes[name]['argv']
    if name == 'planner_node':
        argv = [a.replace('~/goal_pose:=/spot1/imgnav_waypoint', '~/goal_pose:=/spot1/graphnav_destination') for a in argv]
    launch(argv, name)
launch([str(ws / '.venv/bin/python'), str(ws / 'install/wildos_nav2_adapter/lib/wildos_nav2_adapter/graph_path_bridge'), '--ros-args', '-p', 'follow_path:=false'], 'graph_path_bridge')
print('DEPLOYED; control disabled; BeamNG and WildOS process retained', flush=True)
