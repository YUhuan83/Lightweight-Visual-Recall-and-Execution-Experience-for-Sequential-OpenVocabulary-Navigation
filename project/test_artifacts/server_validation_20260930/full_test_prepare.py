"""Prepare one fresh vision process on the existing loaded Town map."""
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
import zipfile
import yaml
from beamngpy import BeamNGpy

ws = Path('/home/ly/beamng_ros2_0394_ws')
out = Path('/home/ly/full_town_memory_20260930')
out.mkdir(exist_ok=True)
stage = out / 'stage'
with zipfile.ZipFile('/home/ly/full_town_memory_20260930.zip') as z:
    z.extractall(stage)
backup = out / 'backup'
backup.mkdir(exist_ok=True)
procs = {}
saved = json.loads(backup.joinpath('processes.json').read_text()) if backup.joinpath('processes.json').exists() else {}
for label, pid in [('vision', 1575639), ('triangulation', 20762), ('control', 2216095), ('graph', 2236112)]:
    proc = Path('/proc', str(pid), 'cmdline')
    args = proc.read_bytes().decode().split('\0')[:-1] if proc.exists() else saved[label]['args']
    procs[label] = {'pid': pid, 'args': args}
backup.joinpath('processes.json').write_text(json.dumps(procs, indent=2))
vision_env = dict(os.environ)
env_path = Path('/proc/1575639/environ')
if not env_path.exists():
    env_path = Path('/proc/20762/environ')
original_env = dict(e.split('=', 1) for e in env_path.read_bytes().decode().split('\0') if '=' in e)
for name in ('PYTHONPATH', 'LD_LIBRARY_PATH', 'CUDA_VISIBLE_DEVICES', 'HF_HOME', 'HF_HUB_OFFLINE', 'TRANSFORMERS_OFFLINE'):
    if name in original_env:
        vision_env[name] = original_env[name]
vision_env.update(ROS_DOMAIN_ID='40', ROS_LOCALHOST_ONLY='1')


def stop(label):
    pid = procs[label]['pid']
    if not Path('/proc', str(pid)).exists():
        return
    os.kill(pid, signal.SIGINT)
    deadline = time.monotonic() + 20
    while Path('/proc', str(pid)).exists() and time.monotonic() < deadline:
        time.sleep(.2)
    if Path('/proc', str(pid)).exists():
        os.kill(pid, signal.SIGTERM)
        deadline = time.monotonic() + 10
        while Path('/proc', str(pid)).exists() and time.monotonic() < deadline:
            time.sleep(.2)
    if Path('/proc', str(pid)).exists():
        raise RuntimeError('Node did not exit after SIGTERM: ' + label)
    print('Stopped', label, flush=True)


def launch(args, label, env=None):
    with out.joinpath(label + '.log').open('ab') as f:
        p = subprocess.Popen(args, cwd=ws, env=env, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
    out.joinpath(label + '.pid').write_text(str(p.pid))
    print('Started', label, p.pid, flush=True)
    return p


b = BeamNGpy('127.0.0.1', 25252)
b.open(launch=False)
b.control.pause()
if str(b.scenario.get_current().level) != 'east_coast_usa':
    raise RuntimeError('Wrong map')
for name in ('vision', 'triangulation', 'graph', 'control'):
    stop(name)
adapter = ws / 'src/wildos_nav2_adapter'
shutil.copytree(adapter, backup / 'wildos_nav2_adapter', symlinks=True)
visual = ws / 'src/nebula2-wildos/visual_navigation'
for name in ('visual_navigation/wildos/nav.py', 'visual_navigation/utils/object_search_utils.py',
             'visual_navigation/explorfm_triangulation/obj_mask_triangulation.py'):
    target = backup / name
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(visual / name, target)
shutil.copy2(ws / 'src/nebula2-wildos/object_search_msgs/msg/ObjectMaskWithTf.msg', backup / 'ObjectMaskWithTf.msg')
shutil.copytree(stage / 'nav2_adapter', adapter, dirs_exist_ok=True)
shutil.copytree(stage / 'wildos_port/repro_check/src/nebula2-wildos/visual_navigation', visual, dirs_exist_ok=True)
shutil.copytree(stage / 'wildos_port/repro_check/src/nebula2-wildos/object_search_msgs',
                ws / 'src/nebula2-wildos/object_search_msgs', dirs_exist_ok=True)
subprocess.run(['colcon', 'build', '--packages-select', 'object_search_msgs', 'wildos_nav2_adapter',
                '--symlink-install', '--executor', 'sequential'], cwd=ws, check=True)
config_dir = ws / 'install/visual_navigation/share/visual_navigation/configs'
cfg = yaml.safe_load((config_dir / 'beamng_wildos_town_return.yaml').read_text())
cfg.update(syncsub_queue_size=10, syncsub_slop=.2, publish_legacy_remembered_goal=False)
cfg['object_search_config']['text_queries'] = ['gas station', 'trash can']
(config_dir / 'beamng_wildos_town_full_20260930.yaml').write_text(yaml.safe_dump(cfg))
tcfg = yaml.safe_load((config_dir / 'triangulation_town_return.yaml').read_text())
tcfg.update(syncsub_queue_size=10, syncsub_slop=.2)
(config_dir / 'triangulation_town_full_20260930.yaml').write_text(yaml.safe_dump(tcfg))
out.joinpath('vision_config.yaml').write_text(yaml.safe_dump(cfg))
out.joinpath('triangulation_config.yaml').write_text(yaml.safe_dump(tcfg))
rot = (0., 0., .7658087977066911, .6430683364581344)
b.vehicles.teleport('ego', (734.294, -11.946, 52.154), rot, reset=True)
v = b.vehicles.get_current()['ego']
v.connect(b)
v.ai.set_mode('disabled')
v.set_shift_mode('realistic_automatic')
v.control(throttle=0, brake=1, parkingbrake=0, steering=0, clutch=0, gear=2)
v.sensors.poll()
out.joinpath('initial_native_state.json').write_text(json.dumps(dict(v.state), indent=2))
exe = ws / 'install/wildos_nav2_adapter/lib/wildos_nav2_adapter'
launch([str(ws / '.venv/bin/python'), str(exe / 'native_vehicle_control')], 'native_control')
args = [a.replace('control_topic:=/control', 'control_topic:=/wildos/native_control') for a in procs['control']['args']]
args.extend(['-p', 'state_topic:=/wildos/native_state'])
launch(args, 'speed_control')
launch(procs['graph']['args'], 'graph_bridge')
args = [a.replace('beamng_wildos_town_return', 'beamng_wildos_town_full_20260930') for a in procs['vision']['args']]
launch(args, 'vision', vision_env)
args = [a.replace('triangulation_town_return.yaml', 'triangulation_town_full_20260930.yaml') for a in procs['triangulation']['args']]
launch(args, 'triangulation', vision_env)
b.control.resume()  # Full brakes remain active; no planning execution is enabled.
v.disconnect()
b.disconnect()
print('Prepared fresh same-process session; map retained; speed and FollowPath disabled', flush=True)
