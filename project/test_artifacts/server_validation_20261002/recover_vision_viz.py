"""Repair late-subscriber visualization and restart only the crashed vision node."""
from pathlib import Path
import json
import shutil
import subprocess
import sys
import time
sys.path.insert(0, '/home/ly')
from session_tools_20261002 import ROOT, WS, runtime_env

target = WS/'src/nebula2-wildos/visual_navigation/visual_navigation/wildos/viz.py'
backup = ROOT/'navigation_repair/viz_before_late_subscription.py'
if not backup.exists():
    shutil.copy2(target, backup)
shutil.copy2('/home/ly/viz_late_subscription_fixed.py', target)
args = json.loads(Path('/home/ly/full_town_memory_20260930/backup/processes.json').read_text())['vision']['args']
args[args.index('--config')+1] = 'beamng_wildos_town_full_20260930'
env = runtime_env()
env['PYTHONPATH'] = str(WS/'src/nebula2-wildos/visual_navigation') + ':/home/ly/wildos-sim/wildos_port/repro_check/src/nebula2-wildos:' + env.get('PYTHONPATH', '')
env.update(HF_HUB_OFFLINE='1', TRANSFORMERS_OFFLINE='1')
with (ROOT/'vision.log').open('ab') as log:
    process = subprocess.Popen(args, cwd=WS, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
(ROOT/'vision.pid').write_text(str(process.pid))
time.sleep(5)
if process.poll() is not None:
    raise RuntimeError('Vision restart failed; inspect vision.log')
print('Recovered vision', process.pid, 'without reloading BeamNG map', flush=True)
