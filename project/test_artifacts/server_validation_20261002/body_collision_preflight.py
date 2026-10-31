"""Read paths, bridge startup and existing sensor inventory; no physics edits."""
from pathlib import Path
import json
from beamngpy import BeamNGpy

root=Path('/home/ly/full_town_memory_20261002/body_collision')
root.mkdir(exist_ok=True)
processes=[]
for p in Path('/proc').iterdir():
    if not p.name.isdigit():continue
    try:
        args=p.joinpath('cmdline').read_bytes().decode().split('\0')[:-1]
        if not args:continue
        if any('beamng_bridge' in a or 'attach' in a and a.endswith('.py') for a in args):
            processes.append({'pid':int(p.name),'args':args,'cwd':str(p.joinpath('cwd').resolve())})
    except (OSError,UnicodeError):pass
b=BeamNGpy('127.0.0.1',25252,socket_timeout=8);b.open(launch=False)
try:
    lua='''
local veh=scenetree.findObject('ego')
local out={userPath=FS:getUserPath(),gamePath=FS:getGamePath(),vehicleID=veh:getID()}
local vm=core_vehicle_manager.getVehicleData(veh:getID())
out.config=vm.config
out.activeCameras=extensions.tech_sensors.getActiveCameraSensors()
out.activeLidars=extensions.tech_sensors.getActiveLidarSensors()
return jsonEncode(out)
'''
    result=json.loads(b.control.queue_lua_command(lua,response=True))
    result['processes']=processes
    (root/'preflight.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result),flush=True)
finally:b.disconnect()
