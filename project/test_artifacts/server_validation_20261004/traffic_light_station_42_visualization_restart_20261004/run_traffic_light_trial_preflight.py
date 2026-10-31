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

def probe_rgb_target():
    probe_code = "\n".join([
        "import json,time,rclpy",
        "from rclpy.node import Node",
        "from rclpy.qos import QoSProfile,ReliabilityPolicy",
        "from sensor_msgs.msg import Image",
        "class Probe(Node):",
        " def __init__(self):",
        "  super().__init__('rgb_target_preflight')",
        "  self.count={'camera':0,'rgb_target':0}",
        "  q=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)",
        "  self.create_subscription(Image,'/spot1/realsense/front/color/image_raw',lambda m:self.bump('camera'),q)",
        "  self.create_subscription(Image,'/wildos/viz/rgb_target',lambda m:self.bump('rgb_target'),q)",
        " def bump(self,key): self.count[key]+=1",
        "rclpy.init();n=Probe();end=time.monotonic()+4",
        "exec('while time.monotonic()<end:\\n rclpy.spin_once(n,timeout_sec=0.1)')",
        "print(json.dumps(n.count));n.destroy_node();rclpy.shutdown()",
    ])
    probe_path=out/'rgb_target_probe.py'
    probe_path.write_text(probe_code+'\n')
    result=subprocess.run([str(WS/'.venv/bin/python'),str(probe_path)],cwd=WS,env=env,
                          capture_output=True,text=True,timeout=9)
    rows=[line for line in result.stdout.splitlines() if line.startswith('{')]
    if result.returncode or not rows:
        raise RuntimeError(f'RGB+Target preflight failed: {result.stdout[-1000:]} {result.stderr[-1000:]}')
    return json.loads(rows[-1])

def restart_image_bridge():
    bridge_exe=WS/'install/wildos_nav2_adapter/lib/wildos_nav2_adapter/visualization_bridge'
    python_exe=WS/'.venv/bin/python'
    found=[]
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            argv=[part.decode(errors='replace') for part in (proc/'cmdline').read_bytes().split(b'\0') if part]
        except (FileNotFoundError,PermissionError,ProcessLookupError):
            continue
        if len(argv)>=2 and argv[0]==str(python_exe) and argv[1]==str(bridge_exe):
            found.append(int(proc.name))
    for pid in found:
        os.kill(pid,signal.SIGTERM)
        for _ in range(40):
            if not Path(f'/proc/{pid}').exists():
                break
            time.sleep(.25)
        if Path(f'/proc/{pid}').exists():
            raise RuntimeError(f'visualization_bridge PID {pid} did not stop cleanly')
    log_path=out/'visualization_bridge.log'
    with log_path.open('w') as log:
        proc=subprocess.Popen([str(python_exe),str(bridge_exe)],cwd=WS,env=env,
                              stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    (out/'visualization_bridge.pid').write_text(str(proc.pid))
    time.sleep(1)
    if proc.poll() is not None:
        raise RuntimeError(f'visualization_bridge exited during startup: {log_path.read_text()[-2000:]}')
    return proc.pid

before_counts=probe_rgb_target()
bridge_restarted=False
bridge_pid=None
after_counts=before_counts
if before_counts.get('rgb_target',0)<3:
    if before_counts.get('camera',0)<3:
        raise RuntimeError(f'camera topic is not live; refusing to record a frozen RGB+Target panel: {before_counts}')
    bridge_pid=restart_image_bridge()
    bridge_restarted=True
    after_counts=probe_rgb_target()
    if after_counts.get('rgb_target',0)<3:
        raise RuntimeError(f'RGB+Target stream stayed stale after bridge restart: before={before_counts}, after={after_counts}')
(out/'visualization_preflight.json').write_text(json.dumps({
    'before':before_counts,'after':after_counts,
    'bridge_restarted':bridge_restarted,'bridge_pid':bridge_pid
},indent=2)+'\n')
print(json.dumps({'visualization_preflight':{
    'before':before_counts,'after':after_counts,
    'bridge_restarted':bridge_restarted,'bridge_pid':bridge_pid
}}),flush=True)
if os.environ.get('WILDOS_PREFLIGHT_ONLY') == '1':
    raise SystemExit(0)
with (out/'rviz_recording.log').open('w') as log:
    recorder = subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','warning','-f','x11grab',
        '-filter_threads','1','-video_size','1280x1024','-framerate','15','-i',':99',
        '-t','1100','-c:v','libx264','-threads','1','-preset','ultrafast','-pix_fmt','yuv420p',
        str(out/'rviz_live.mp4')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
(out/'recorder.pid').write_text(str(recorder.pid))
time.sleep(.5)
if recorder.poll() is not None:
    raise RuntimeError((out/'rviz_recording.log').read_text())
trial = None
try:
    with (out/'trial.log').open('w') as log:
        trial = subprocess.Popen([str(WS/'.venv/bin/python'),str(ROOT/'traffic_light_station_test.py'),name],
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
