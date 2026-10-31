"""Restore the sole native actuator if a diagnostic socket has timed out."""
from pathlib import Path
import subprocess
import sys
import time
sys.path.insert(0,'/home/ly')
from session_tools_20261002 import ROOT,WS,runtime_env
pid=int((ROOT/'native_control.pid').read_text());proc=Path('/proc',str(pid))
if proc.exists() and proc.joinpath('stat').read_text().split()[2]!='Z':raise RuntimeError('Native node is already running')
args=[str(WS/'.venv/bin/python'),str(WS/'install/wildos_nav2_adapter/lib/wildos_nav2_adapter/native_vehicle_control'),
      '--ros-args','-p','overspeed_limit:=0.0','-p','command_timeout:=0.8']
with (ROOT/'navigation_repair/native_control.log').open('ab') as log:
    p=subprocess.Popen(args,cwd=WS,env=runtime_env(),stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
(ROOT/'native_control.pid').write_text(str(p.pid));time.sleep(1)
if p.poll() is not None:raise RuntimeError('Native recovery failed')
print('Native control restored',p.pid,flush=True)
