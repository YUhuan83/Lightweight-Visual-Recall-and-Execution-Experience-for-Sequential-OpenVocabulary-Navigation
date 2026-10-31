"""Update sources and test planning without restarting the map or WildOS."""
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

ws = Path('/home/ly/beamng_ros2_0394_ws')
stage = Path('/home/ly/memory_revision_20260930_stage')
backup = Path('/home/ly/memory_revision_20260930_backup')
logs = Path('/home/ly/navigation_validation_20260930_logs')
bridge = str(ws / 'install/wildos_nav2_adapter/lib/wildos_nav2_adapter/graph_path_bridge')
if backup.exists():
    raise RuntimeError('Inspect existing memory revision backup before another deployment')
backup.mkdir()
processes = []
for p in Path('/proc').iterdir():
    if p.name.isdigit():
        try:
            args = p.joinpath('cmdline').read_bytes().decode().split('\0')[:-1]
        except (OSError, UnicodeError):
            continue
        if bridge in args:
            processes.append({'pid': int(p.name), 'args': args})
if len(processes) != 1 or 'follow_path:=false' not in processes[0]['args']:
    raise RuntimeError('Expected exactly one planning-only bridge')
backup.joinpath('bridge.json').write_text(json.dumps(processes, indent=2))
adapter = ws / 'src/wildos_nav2_adapter'
visual = ws / 'src/nebula2-wildos/visual_navigation'
shutil.copytree(adapter, backup / 'wildos_nav2_adapter', symlinks=True)
shutil.copy2(visual / 'visual_navigation/wildos/nav.py', backup / 'nav.py')
shutil.copytree(stage / 'nav2_adapter', adapter, dirs_exist_ok=True)
for name in ('nav.py', 'viewpoint_memory.py'):
    shutil.copy2(stage / 'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/wildos' / name,
                 visual / 'visual_navigation/wildos' / name)
subprocess.run(['colcon', 'build', '--packages-select', 'wildos_nav2_adapter', '--symlink-install'], cwd=ws, check=True)
# The original model process keeps its existing in-process memory. Its new
# producer code becomes active on its next normal startup.
os.kill(processes[0]['pid'], signal.SIGINT)
deadline = time.monotonic() + 10
while Path('/proc', str(processes[0]['pid'])).exists() and time.monotonic() < deadline:
    time.sleep(.1)
if Path('/proc', str(processes[0]['pid'])).exists():
    raise RuntimeError('Old planning bridge did not stop')
try:
    result = subprocess.run([str(ws / '.venv/bin/python'), '/home/ly/memory_revision_20260930_probe.py'], timeout=250)
    if result.returncode:
        raise RuntimeError('Planning probe failed: ' + str(result.returncode))
finally:
    with logs.joinpath('memory_revision_bridge.log').open('ab') as log:
        node = subprocess.Popen(processes[0]['args'], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    logs.joinpath('memory_revision_bridge.pid').write_text(str(node.pid))
    print('Restored planning-only bridge:', node.pid, flush=True)
print('Original WildOS retained:', Path('/proc/1575639').exists(), flush=True)
