"""Restart an unresponsive route execution bridge while the simulation is paused."""
from pathlib import Path
import os
import signal
import subprocess
import sys
import time
sys.path.insert(0, '/home/ly')
from session_tools_20261002 import ROOT, WS, runtime_env
name=sys.argv[1] if len(sys.argv)>1 else 'graph_bridge'
if name not in ('graph_bridge','goal_standoff'):raise ValueError(name)
pid=int((ROOT/(name+'.pid')).read_text());proc=Path('/proc',str(pid))
args=proc.joinpath('cmdline').read_bytes().decode().split('\0')[:-1]
env=dict(v.split('=',1) for v in proc.joinpath('environ').read_bytes().decode().split('\0') if '=' in v)
os.kill(pid,signal.SIGINT)
end=time.monotonic()+15
while proc.exists() and time.monotonic()<end:
    if proc.joinpath('stat').read_text().split()[2]=='Z':break
    time.sleep(.1)
if proc.exists() and proc.joinpath('stat').read_text().split()[2]!='Z':raise RuntimeError('Bridge did not stop')
with (ROOT/('navigation_repair/graph_bridge.log' if name=='graph_bridge' else 'goal_standoff.log')).open('ab') as log:
    p=subprocess.Popen(args,cwd=WS,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
(ROOT/(name+'.pid')).write_text(str(p.pid));time.sleep(2)
if p.poll() is not None:raise RuntimeError('Bridge startup failed')
print(name,'restarted',pid,'->',p.pid,flush=True)
