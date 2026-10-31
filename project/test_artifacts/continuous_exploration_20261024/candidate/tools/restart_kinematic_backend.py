import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
root=Path('/home/ly/town_memory_validation_20261003/physics_isolation_20261006')
pid=int((root/'kinematic.pid').read_text())
procpath=Path('/proc',str(pid))
if procpath.exists():
    assert b'wildos_nav2_adapter.kinematic_vehicle_control' in (procpath/'cmdline').read_bytes()
    os.kill(pid,signal.SIGINT)
    end=time.monotonic()+10
    while procpath.exists() and 'State:\tZ' not in (procpath/'status').read_text():
        assert time.monotonic()<end, 'Old backend did not stop'
        time.sleep(.1)
env=dict(os.environ)
env.update(json.loads(Path('/home/ly/full_town_memory_20261002/runtime_env.json').read_text()))
with (root/'kinematic.log').open('a') as log:
    proc=subprocess.Popen([sys.executable,'-m','wildos_nav2_adapter.kinematic_vehicle_control',
        '--ros-args','-p','enable:=false'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
(root/'kinematic.pid').write_text(str(proc.pid))
time.sleep(1)
assert proc.poll() is None
rows=json.loads((root/'process_manifest.json').read_text())
row=rows['official_adapter']
pid_file=root.parent/'official_adapter.pid'
adapter_pid=int(pid_file.read_text())
adapter_path=Path('/proc',str(adapter_pid))
if adapter_path.exists():
    assert row['args'][1].encode() in (adapter_path/'cmdline').read_bytes()
    os.kill(adapter_pid,signal.SIGINT)
    end=time.monotonic()+10
    while adapter_path.exists() and 'State:\tZ' not in (adapter_path/'status').read_text():
        assert time.monotonic()<end
        time.sleep(.1)
with (root/'official_adapter.log').open('a') as log:
    adapter=subprocess.Popen(row['args']+['--ros-args','-r','/vehicles/ego/sensors/state:=/wildos/kinematic_state'],
        env=env,cwd=row['cwd'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
pid_file.write_text(str(adapter.pid))
print(json.dumps({'pid':proc.pid,'enable':False}))
