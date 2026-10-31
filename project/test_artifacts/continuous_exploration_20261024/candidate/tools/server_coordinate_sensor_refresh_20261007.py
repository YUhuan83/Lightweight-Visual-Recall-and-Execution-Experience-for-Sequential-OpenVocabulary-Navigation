"""Restart the identified stale sensor owner on the same loaded map."""
import os,json,signal,subprocess,time
from pathlib import Path
root=Path('/home/ly/coordinate_probe_20261007');root.mkdir(exist_ok=True)
path=Path('/proc/2593885')
if path.exists():
    assert b'attach_fixed_vehicle_ideal_20261006.py' in (path/'cmdline').read_bytes()
    os.kill(2593885,signal.SIGINT)
    end=time.monotonic()+5
    while path.exists() and 'State:\tZ' not in (path/'status').read_text() and time.monotonic()<end:time.sleep(.1)
    if path.exists() and 'State:\tZ' not in (path/'status').read_text():os.kill(2593885,signal.SIGKILL)
original=Path('/home/ly/town_memory_validation_20261003/physics_isolation_20261006/attach_fixed_vehicle_ideal_20261006.py').read_text()
(root/'sensor_bridge.before.py').write_text(original)
source=original.replace("profile={'known_vehicle_port': 39519}\nvehicle.connection=Connection('127.0.0.1',int(profile['known_vehicle_port']),timeout=8)",
    "b.control.queue_lua_command(\"local v=scenetree.findObject('ego'); extensions.tech_sensors.removeAllSensorsFromVehicle(v:getID()); return true\",response=True)\nvehicle.connection=Connection('127.0.0.1',39519,timeout=8)")
assert source!=original
script=root/'attach_sensor_bridge.py';script.write_text(source)
env=dict(os.environ);env.update(json.loads(Path('/home/ly/full_town_memory_20261002/runtime_env.json').read_text()))
with (root/'sensor_bridge.log').open('a') as log:
    child=subprocess.Popen(['/home/ly/beamng_ros2_0394_ws/.venv/bin/python',str(script)],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
(root/'sensor_bridge.pid').write_text(str(child.pid));time.sleep(3)
assert child.poll() is None,'New sensor bridge failed'
print(json.dumps({'sensor_pid':child.pid}))
