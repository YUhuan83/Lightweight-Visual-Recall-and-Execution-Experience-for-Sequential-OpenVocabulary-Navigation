"""Own the planner debugger so Yama need not be changed; retain Town."""
from pathlib import Path
import json
import os
import signal
import subprocess
import time

ROOT = Path('/home/ly/town_memory_validation_20261003')
rows = {r['name']: r for r in json.loads((ROOT/'restart_snapshot.json').read_text())}

def stop(name):
    row = rows[name]
    pid = int((ROOT/(name+'.pid')).read_text())
    proc = Path('/proc')/str(pid)
    if (proc/'cmdline').exists():
        assert row['args'][0].encode() in (proc/'cmdline').read_bytes()
        for sig, seconds in ((signal.SIGINT, 4), (signal.SIGTERM, 2), (signal.SIGKILL, 2)):
            if not proc.exists() or 'State:\tZ' in (proc/'status').read_text():
                break
            os.kill(pid, sig)
            deadline = time.monotonic()+seconds
            while proc.exists() and time.monotonic()<deadline:
                if 'State:\tZ' in (proc/'status').read_text():
                    break
                time.sleep(.1)
    print(json.dumps({'stopped': name, 'pid': pid}), flush=True)

for name in ('lifecycle', 'controller', 'planner'):
    stop(name)

row = rows['planner']
env = dict(row['environment'], ROS_DOMAIN_ID='40', ROS_LOCALHOST_ONLY='1', ROS2CLI_NO_DAEMON='1')
trace = (ROOT/'planner_block_gdb.log').open('w')
debugger = subprocess.Popen(['gdb', '-q', '--args']+row['args'], cwd=row['cwd'], env=env,
    stdin=subprocess.PIPE, stdout=trace, stderr=subprocess.STDOUT, text=True, start_new_session=True)
debugger.stdin.write('set pagination off\nset confirm off\nrun\n')
debugger.stdin.flush()
time.sleep(2)
children = (Path('/proc')/str(debugger.pid)/'task'/str(debugger.pid)/'children').read_text().split()
inferiors = [int(p) for p in children if b'planner_server' in (Path('/proc')/p/'cmdline').read_bytes()]
assert len(inferiors)==1, 'Planner debugger did not start one inferior'
(ROOT/'planner.pid').write_text(str(inferiors[0]))
print(json.dumps({'debugger': debugger.pid, 'planner': inferiors[0]}), flush=True)
for name in ('controller', 'lifecycle'):
    subprocess.run([os.sys.executable, str(ROOT/'restart_component.py'), name], check=True, timeout=25)
time.sleep(3)
try:
    subprocess.run([os.sys.executable, str(ROOT/'diagnose_planner_readiness.py'), '--clear'],
                   env=env, check=True, timeout=40)
    os.kill(debugger.pid, signal.SIGINT)
    time.sleep(1)
    debugger.stdin.write('thread apply all bt 15\n')
    debugger.stdin.flush()
    time.sleep(2)
finally:
    # Leave the normal planner running with no debugger attached.
    debugger.stdin.write('detach\nquit\n')
    debugger.stdin.flush()
    debugger.wait(timeout=10)
    trace.close()
print(json.dumps({'stack_trace': (ROOT/'planner_block_gdb.log').read_text()[-24000:]}), flush=True)
