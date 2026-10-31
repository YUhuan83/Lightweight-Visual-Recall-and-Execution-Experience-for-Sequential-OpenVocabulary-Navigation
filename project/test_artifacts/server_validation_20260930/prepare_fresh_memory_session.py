"""Clear trial history before the final sequence, retaining the loaded map."""
import os
from pathlib import Path
import signal
import subprocess
import time
from beamngpy import BeamNGpy

out=Path('/home/ly/full_town_memory_20260930')
pid=int(out.joinpath('vision.pid').read_text())
args=Path('/proc',str(pid),'cmdline').read_bytes().decode().split('\0')[:-1]
env=dict(e.split('=',1) for e in Path('/proc',str(pid),'environ').read_bytes().decode().split('\0') if '=' in e)
b=BeamNGpy('127.0.0.1',25252)
b.open(launch=False)
b.control.pause()
assert str(b.scenario.get_current().level)=='east_coast_usa'
b.vehicles.teleport('ego',(734.294,-11.946,52.154),(0.,0.,.7658087977066911,.6430683364581344),reset=True)
b.disconnect()
os.kill(pid,signal.SIGINT)
deadline=time.monotonic()+20
while Path('/proc',str(pid)).exists() and time.monotonic()<deadline:time.sleep(.2)
if Path('/proc',str(pid)).exists():raise RuntimeError('Old vision process did not exit')
with out.joinpath('vision_fresh.log').open('ab') as f:
    p=subprocess.Popen(args,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
out.joinpath('vision.pid').write_text(str(p.pid))
print('Fresh vision PID before final test:',p.pid,'map retained; no restart between search goals',flush=True)
