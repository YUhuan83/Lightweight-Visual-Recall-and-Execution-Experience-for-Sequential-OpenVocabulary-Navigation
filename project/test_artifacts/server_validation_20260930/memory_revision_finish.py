"""Apply the final bridge edit and recheck occupied-goal fallback."""
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

ws = Path('/home/ly/beamng_ros2_0394_ws')
stage = Path('/home/ly/memory_revision_20260930_stage')
logs = Path('/home/ly/navigation_validation_20260930_logs')
bridge = str(ws / 'install/wildos_nav2_adapter/lib/wildos_nav2_adapter/graph_path_bridge')
pid = int(logs.joinpath('memory_revision_bridge.pid').read_text())
args = Path('/proc', str(pid), 'cmdline').read_bytes().decode().split('\0')[:-1]
if bridge not in args or 'follow_path:=false' not in args:
    raise RuntimeError('Expected retained planning-only bridge')
shutil.copy2(stage / 'nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py',
             ws / 'src/wildos_nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py')
os.kill(pid, signal.SIGINT)
deadline = time.monotonic() + 10
while Path('/proc', str(pid)).exists() and time.monotonic() < deadline:
    time.sleep(.1)
if Path('/proc', str(pid)).exists():
    raise RuntimeError('Bridge did not stop')
try:
    result = subprocess.run([str(ws / '.venv/bin/python'), '/home/ly/memory_revision_20260930_probe.py'], timeout=250)
finally:
    with logs.joinpath('memory_revision_bridge.log').open('ab') as log:
        node = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    logs.joinpath('memory_revision_bridge.pid').write_text(str(node.pid))
    print('Final planning-only bridge:', node.pid, flush=True)
raise SystemExit(result.returncode)
