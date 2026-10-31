"""Run one current-map memory trial and record the existing RViz display."""
from pathlib import Path
import os
import signal
import subprocess
import sys
import time
sys.path.insert(0, '/home/ly')
from session_tools_20261002 import ROOT, WS, runtime_env

name = sys.argv[1]
out = ROOT/name
if out.exists():
    raise RuntimeError('Refusing to overwrite a previous trial')
out.mkdir()
env = runtime_env()
if name.startswith('body_collision_'):
    env['WILDOS_REQUIRE_BODY_COLLISION_DISABLED'] = '1'
rviz_pid = int((ROOT/'rviz.pid').read_text())
if not Path('/proc', str(rviz_pid)).exists():
    raise RuntimeError('RViz is not running')
with (out/'rviz_recording.log').open('w') as log:
    recorder = subprocess.Popen(['ffmpeg', '-hide_banner', '-loglevel', 'warning',
        '-f', 'x11grab', '-filter_threads', '1', '-video_size', '1280x1024',
        '-framerate', '5', '-i', ':99', '-t', '1100', '-c:v', 'libx264',
        '-threads', '1', '-preset', 'ultrafast', '-pix_fmt', 'yuv420p', str(out/'rviz_live.mp4')],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
(out/'recorder.pid').write_text(str(recorder.pid))
time.sleep(.5)
if recorder.poll() is not None:
    raise RuntimeError('RViz recording failed')
trial = None
try:
    with (out/'trial.log').open('w') as log:
        trial = subprocess.Popen([str(WS/'.venv/bin/python'), str(ROOT/'full_town_memory_test.py'), name],
            cwd=WS, env=env, stdout=log, stderr=subprocess.STDOUT)
        (out/'trial.pid').write_text(str(trial.pid))
        print('Started', name, 'PID', trial.pid, 'RViz recorder', recorder.pid, flush=True)
        result = trial.wait()
    print('Trial exit', result, flush=True)
    if result:
        print((out/'trial.log').read_text()[-2500:], flush=True)
    raise SystemExit(result)
finally:
    if trial is not None and trial.poll() is None:
        trial.send_signal(signal.SIGINT)
        try:
            trial.wait(timeout=20)
        except subprocess.TimeoutExpired:
            subprocess.run([str(WS/'.venv/bin/python'), '/home/ly/session_tools_20261002.py',
                            'inspect', 'interrupted_trial'], env=env, timeout=30)
    if recorder.poll() is None:
        recorder.send_signal(signal.SIGINT)
        try:
            recorder.wait(timeout=10)
        except subprocess.TimeoutExpired:
            recorder.terminate()
            recorder.wait(timeout=5)
