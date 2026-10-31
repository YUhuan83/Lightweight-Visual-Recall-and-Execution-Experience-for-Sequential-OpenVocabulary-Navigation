import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from beamngpy import BeamNGpy

root=Path('/home/ly/town_memory_validation_20261003/physics_isolation_20261006')
pid=int((root/'kinematic.pid').read_text())
process=Path('/proc',str(pid))
if process.exists():
    assert b'wildos_nav2_adapter.kinematic_vehicle_control' in (process/'cmdline').read_bytes()
    os.kill(pid,signal.SIGINT)
    until=time.monotonic()+8
    while process.exists() and 'State:\tZ' not in (process/'status').read_text():
        assert time.monotonic()<until
        time.sleep(.1)
env=dict(os.environ)
env.update(json.loads(Path('/home/ly/full_town_memory_20261002/runtime_env.json').read_text()))
b=BeamNGpy('127.0.0.1',25252,socket_timeout=8).open(launch=False)
try:
    source=Path('/home/ly/wildosKinematic.lua').read_text()
    literal=json.dumps(source)
    b.control.queue_lua_command('local s='+literal+"; assert(loadstring(s)); writeFile('lua/ge/extensions/wildosKinematic.lua',s); "
        "extensions.unload('wildosKinematic'); extensions.load('wildosKinematic'); return true",response=True)
    b.control.pause()
finally:
    b.disconnect()
with (root/'kinematic.log').open('a') as log:
    new=subprocess.Popen([sys.executable,'-m','wildos_nav2_adapter.kinematic_vehicle_control',
        '--ros-args','-p','enable:=true'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
(root/'kinematic.pid').write_text(str(new.pid))
time.sleep(1)
assert new.poll() is None
print(json.dumps({'pid':new.pid,'enable':True}))
