"""Back up and refresh only the known ideal motion backend for this test."""
import os,json,signal,subprocess,time,shutil
from pathlib import Path
root=Path('/home/ly/coordinate_probe_20261007');root.mkdir(exist_ok=True)
target=Path('/home/ly/beamng_ros2_0394_ws/src/wildos_nav2_adapter/wildos_nav2_adapter/kinematic_vehicle_control.py')
shutil.copy2(target,root/'kinematic_vehicle_control.before.py')
pid=2580232
process=Path('/proc')/str(pid)
if process.exists():
    cmd=(process/'cmdline').read_bytes()
    assert b'/lib/wildos_nav2_adapter/kinematic_vehicle_control\x00' in cmd
    (root/'backend_command_before.json').write_text(json.dumps([x.decode() for x in cmd.split(b'\x00') if x]))
    os.kill(pid,signal.SIGINT)
    end=time.monotonic()+10
    while process.exists() and 'State:\tZ' not in (process/'status').read_text():
        assert time.monotonic()<end,'Backend did not stop'
        time.sleep(.1)
shutil.copy2('/home/ly/kinematic_vehicle_control_20261007.py',target)
env=dict(os.environ);env.update(json.loads(Path('/home/ly/full_town_memory_20261002/runtime_env.json').read_text()))
with (root/'backend.log').open('a') as log:
    child=subprocess.Popen(['/home/ly/beamng_ros2_0394_ws/.venv/bin/python','-m','wildos_nav2_adapter.kinematic_vehicle_control','--ros-args','-p','enable:=false'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
(root/'backend.pid').write_text(str(child.pid));time.sleep(2)
assert child.poll() is None,'New backend startup failed'
print(json.dumps({'backend_pid':child.pid,'backup':str(root/'kinematic_vehicle_control.before.py')}))
