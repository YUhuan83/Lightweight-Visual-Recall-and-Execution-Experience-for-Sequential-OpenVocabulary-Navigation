"""Capture RViz while testing bus-stop-sign search then station memory return."""
from pathlib import Path
import json
import os
import signal
import subprocess
import sys
import time

ROOT = Path("/home/ly/town_memory_validation_20261003")
WS = Path("/home/ly/beamng_ros2_0394_ws")
name = sys.argv[1] if len(sys.argv) > 1 else "bus_stop_sign_station_arrival_20261005_trial1"
if not name.replace("_", "").isalnum():
    raise ValueError("Invalid test name")
out = ROOT / name
if out.exists():
    raise RuntimeError("Test output already exists: " + str(out))
out.mkdir()
runtime_env = json.loads(Path("/home/ly/full_town_memory_20261002/runtime_env.json").read_text())
env = dict(os.environ, **runtime_env)
env.update(ROS_DOMAIN_ID="40", ROS_LOCALHOST_ONLY="1", ROS2CLI_NO_DAEMON="1",
           WILDOS_TEST_SESSION_DIR=str(ROOT), WILDOS_REQUIRE_BODY_COLLISION_DISABLED="1")

with (out / "rviz_recording.log").open("w") as recorder_log:
    recorder = subprocess.Popen([
        "ffmpeg", "-hide_banner", "-loglevel", "warning", "-f", "x11grab",
        "-filter_threads", "1", "-video_size", "1280x1024", "-framerate", "5",
        "-i", ":99", "-t", "1200", "-c:v", "libx264", "-threads", "1",
        "-preset", "ultrafast", "-pix_fmt", "yuv420p", str(out / "rviz_live.mp4")],
        stdout=recorder_log, stderr=subprocess.STDOUT, start_new_session=True)
(out / "recorder.pid").write_text(str(recorder.pid))
time.sleep(0.7)
if recorder.poll() is not None:
    raise RuntimeError((out / "rviz_recording.log").read_text())

trial = None
try:
    with (out / "trial.log").open("w") as trial_log:
        trial = subprocess.Popen([
            str(WS / ".venv/bin/python"),
            str(ROOT / "bus_stop_sign_station_arrival_test.py"), name],
            cwd=WS, env=env, stdout=trial_log, stderr=subprocess.STDOUT,
            start_new_session=True)
        (out / "trial.pid").write_text(str(trial.pid))
        print(json.dumps({"name": name, "trial_pid": trial.pid,
                          "recorder_pid": recorder.pid}), flush=True)
        result = trial.wait()
    print(json.dumps({"trial_exit": result}), flush=True)
    raise SystemExit(result)
finally:
    if trial is not None and trial.poll() is None:
        trial.send_signal(signal.SIGINT)
        trial.wait(timeout=20)
    if recorder.poll() is None:
        recorder.send_signal(signal.SIGINT)
        try:
            recorder.wait(timeout=15)
        except subprocess.TimeoutExpired:
            recorder.terminate()
            recorder.wait(timeout=5)
