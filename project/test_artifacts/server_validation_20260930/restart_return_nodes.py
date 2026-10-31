"""Restart only the disabled control and route bridge; retain visual memory."""
import os
from pathlib import Path
import signal
import subprocess
import time

out = Path('/home/ly/full_town_memory_20260930')
for label in ('graph_bridge', 'speed_control'):
    pid = int(out.joinpath(label+'.pid').read_text())
    args = Path('/proc',str(pid),'cmdline').read_bytes().decode().split('\0')[:-1]
    if label == 'speed_control':
        args += ['-p','allow_reverse:=true']
    os.kill(pid,signal.SIGINT)
    deadline = time.monotonic()+8
    while Path('/proc',str(pid)).exists() and time.monotonic()<deadline:
        time.sleep(.1)
    if Path('/proc',str(pid)).exists():
        raise RuntimeError('Node did not exit: '+label)
    with out.joinpath(label+'.log').open('ab') as stream:
        p = subprocess.Popen(args,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    out.joinpath(label+'.pid').write_text(str(p.pid))
    print(label,p.pid,flush=True)
