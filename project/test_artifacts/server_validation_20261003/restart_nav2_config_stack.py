"""Restart the official odom adapter and Nav2 stack, retaining BeamNG."""
from pathlib import Path
import json
import os
import signal
import subprocess
import time

ROOT = Path('/home/ly/town_memory_validation_20261003')
NAMES = ('lifecycle', 'controller', 'planner', 'official_adapter')
rows = {row['name']: row for row in json.loads((ROOT / 'restart_snapshot.json').read_text())}


def stop(name):
    row = rows[name]
    pid_path = ROOT / (name + '.pid')
    pid = int(pid_path.read_text())
    proc = Path('/proc') / str(pid)
    cmdline = proc / 'cmdline'
    if cmdline.exists():
        identity = row['args'][1] if name == 'official_adapter' else row['args'][0]
        assert identity.encode() in cmdline.read_bytes(), f'{name} PID identity changed'
        for sig, wait in ((signal.SIGINT, 4), (signal.SIGTERM, 3), (signal.SIGKILL, 3)):
            if not cmdline.exists() or 'State:\tZ' in (proc / 'status').read_text():
                break
            os.kill(pid, sig)
            deadline = time.monotonic() + wait
            while cmdline.exists() and time.monotonic() < deadline:
                if 'State:\tZ' in (proc / 'status').read_text():
                    break
                time.sleep(.2)
        if cmdline.exists() and 'State:\tZ' not in (proc / 'status').read_text():
            raise RuntimeError(f'{name} did not stop')
    print(json.dumps({'stopped': name, 'previous_pid': pid}), flush=True)


def start(name):
    row = rows[name]
    args = list(row['args'])
    for i, arg in enumerate(args):
        if arg == '--params-file':
            assert Path(args[i + 1]).is_file(), f'{name} parameter file is missing'
    env = dict(row['environment'])
    env.update(ROS_DOMAIN_ID='40', ROS_LOCALHOST_ONLY='1', ROS2CLI_NO_DAEMON='1')
    with (ROOT / (name + '.log')).open('a') as log:
        process = subprocess.Popen(args, cwd=row['cwd'], env=env, stdout=log,
                                    stderr=subprocess.STDOUT, start_new_session=True)
    (ROOT / (name + '.pid')).write_text(str(process.pid))
    time.sleep(1)
    if process.poll() is not None:
        raise RuntimeError(f'{name} failed to start')
    print(json.dumps({'started': name, 'pid': process.pid}), flush=True)
    time.sleep(2)


for component in NAMES:
    stop(component)
for component in ('official_adapter', 'planner', 'controller', 'lifecycle'):
    start(component)
print('ODOM_ADAPTER_AND_NAV2_RESTARTED; BeamNG process/map untouched', flush=True)
