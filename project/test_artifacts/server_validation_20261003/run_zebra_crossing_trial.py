"""Record RViz and run the current-map, same-process multi-goal trial."""
from pathlib import Path
import json
import os
import signal
import subprocess
import sys
import time

ROOT = Path('/home/ly/town_memory_validation_20261003')
WS = Path('/home/ly/beamng_ros2_0394_ws')
name = sys.argv[1]
if not name.replace('_','').isalnum():
    raise ValueError('Invalid trial name')
out = ROOT/name
if out.exists():
    raise RuntimeError('Trial output already exists')
out.mkdir()
env = dict(os.environ, **json.loads(Path('/home/ly/full_town_memory_20261002/runtime_env.json').read_text()))
env.update(ROS_DOMAIN_ID='40', ROS_LOCALHOST_ONLY='1', ROS2CLI_NO_DAEMON='1',
           WILDOS_TEST_SESSION_DIR=str(ROOT), WILDOS_REQUIRE_BODY_COLLISION_DISABLED='1')
with (out/'rviz_recording.log').open('w') as log:
    recorder = subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','warning','-f','x11grab',
        '-filter_threads','1','-video_size','1280x1024','-framerate','5','-i',':99',
        '-t','1100','-c:v','libx264','-threads','1','-preset','ultrafast','-pix_fmt','yuv420p',
        str(out/'rviz_live.mp4')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
(out/'recorder.pid').write_text(str(recorder.pid))
time.sleep(.5)
if recorder.poll() is not None:
    raise RuntimeError((out/'rviz_recording.log').read_text())
trial = None
try:
    with (out/'trial.log').open('w') as log:
        trial = subprocess.Popen([str(WS/'.venv/bin/python'),str(ROOT/'zebra_crossing_station_test.py'),name],
            cwd=WS,env=env,stdout=log,stderr=subprocess.STDOUT)
        (out/'trial.pid').write_text(str(trial.pid))
        print(json.dumps(dict(name=name,trial_pid=trial.pid,recorder_pid=recorder.pid)),flush=True)
        result=trial.wait()
    print(json.dumps(dict(trial_exit=result)),flush=True)
    raise SystemExit(result)
finally:
    if trial is not None and trial.poll() is None:
        trial.send_signal(signal.SIGINT)
        trial.wait(timeout=20)
    if recorder.poll() is None:
        recorder.send_signal(signal.SIGINT)
        try:recorder.wait(timeout=10)
        except subprocess.TimeoutExpired:
            recorder.terminate()
            recorder.wait(timeout=5)
