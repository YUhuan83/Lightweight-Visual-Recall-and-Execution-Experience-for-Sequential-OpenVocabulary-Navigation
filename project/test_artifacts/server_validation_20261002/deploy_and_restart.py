from pathlib import Path
import hashlib
import json
import os
import shutil
import signal
import subprocess
import sys
import time
import zipfile

sys.path.insert(0, '/home/ly')
from session_tools_20261002 import ROOT, WS, runtime_env

environment = runtime_env()
source = WS / 'src/wildos_nav2_adapter'
backup = ROOT / 'adapter_backup'
if backup.exists():
    raise RuntimeError('Backup already exists; inspect previous deployment before retry')
shutil.copytree(source, backup, symlinks=True)
with zipfile.ZipFile(ROOT / 'adapter_stage.zip') as archive:
    for name in archive.namelist():
        target = (source / name).resolve()
        if not target.is_relative_to(source.resolve()):
            raise RuntimeError('Unsafe archive entry: ' + name)
    archive.extractall(source)
build = subprocess.run(['colcon', 'build', '--packages-select', 'wildos_nav2_adapter',
                        '--symlink-install', '--event-handlers', 'console_direct+'],
                       cwd=WS, env=environment, capture_output=True, text=True, timeout=180)
(ROOT / 'build.log').write_text(build.stdout + build.stderr)
if build.returncode:
    print(build.stdout + build.stderr)
    raise RuntimeError('Adapter build failed')
print('Adapter build complete', flush=True)

old = {'graph_bridge': 2311732, 'speed_control': 2284240, 'native_control': 2249247,
       'planner': 964196, 'controller': 964198, 'lifecycle': 964200}
commands = {}
for name, pid in old.items():
    proc = Path('/proc', str(pid))
    commands[name] = dict(args=proc.joinpath('cmdline').read_bytes().decode().split('\0')[:-1],
                          cwd=str(proc.joinpath('cwd').resolve()))
(ROOT / 'old_commands.json').write_text(json.dumps(commands, indent=2))
for name in ('graph_bridge', 'speed_control', 'native_control', 'lifecycle', 'controller', 'planner'):
    pid = old[name]
    os.kill(pid, signal.SIGINT)
    end = time.monotonic() + 8
    while Path('/proc', str(pid)).exists() and time.monotonic() < end:
        stat = Path('/proc', str(pid), 'stat')
        if stat.exists() and stat.read_text().split()[2] == 'Z':
            break
        time.sleep(.1)
    if Path('/proc', str(pid)).exists() and Path('/proc', str(pid), 'stat').read_text().split()[2] != 'Z':
        raise RuntimeError('Node did not stop: ' + name)

for name in ('planner', 'controller', 'lifecycle', 'native_control', 'speed_control', 'graph_bridge'):
    args = commands[name]['args']
    with (ROOT / (name + '.log')).open('ab') as log:
        process = subprocess.Popen(args, cwd=commands[name]['cwd'], env=environment,
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    (ROOT / (name + '.pid')).write_text(str(process.pid))
    print(name, process.pid, flush=True)
    if name == 'lifecycle':
        time.sleep(4.)
    else:
        time.sleep(.5)

# Run the real ROS classes and messages on an isolated domain. No actuator
# node is present there, and the tests use mocked action/service endpoints.
test_env = dict(environment, ROS_DOMAIN_ID='49')
test = subprocess.run([str(WS / '.venv/bin/python'), '-m', 'unittest', 'discover',
                       '-s', str(source / 'test'), '-p', 'test_graph_path_bridge_routing.py', '-v'],
                      cwd=WS, env=test_env, capture_output=True, text=True, timeout=60)
(ROOT / 'ros_graph_bridge_regression.log').write_text(test.stdout + test.stderr)
test2 = subprocess.run([str(WS / '.venv/bin/python'), '-m', 'unittest', 'discover',
                        '-s', str(source / 'test'), '-p', 'test_cmd_vel_to_control_ros.py', '-v'],
                       cwd=WS, env=test_env, capture_output=True, text=True, timeout=30)
(ROOT / 'ros_control_regression.log').write_text(test2.stdout + test2.stderr)
manifest = {name: hashlib.sha256((source / name).read_bytes()).hexdigest() for name in
            ('wildos_nav2_adapter/graph_path_bridge.py', 'wildos_nav2_adapter/motion_path.py',
             'wildos_nav2_adapter/drive_math.py', 'wildos_nav2_adapter/cmd_vel_to_control.py',
             'wildos_nav2_adapter/native_vehicle_control.py', 'config/nav2_official_control.yaml')}
(ROOT / 'deployment_manifest.json').write_text(json.dumps(manifest, indent=2))
print('ROS graph tests:', test.returncode, 'control tests:', test2.returncode, flush=True)
print((test.stdout + test.stderr)[-800:], flush=True)
if test.returncode or test2.returncode:
    raise RuntimeError('ROS regression failed; driving remains disabled')
