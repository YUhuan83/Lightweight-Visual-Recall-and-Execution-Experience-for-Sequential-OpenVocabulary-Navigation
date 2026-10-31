"""Read the applied profile from GE without opening a vehicle socket."""
from pathlib import Path
import json
from beamngpy import BeamNGpy
root=Path('/home/ly/full_town_memory_20261002/body_collision')
b=BeamNGpy('127.0.0.1',25252,socket_timeout=8);b.open(launch=False)
try:
    lua='''
local veh=scenetree.findObject('ego')
local out=extensions.wildosBodyCollision.status(veh:getID())
local bundle=core_vehicle_manager.getVehicleData(veh:getID())
out.collisionMasks={}
for _,node in pairs(bundle.vdata.nodes) do
  local mask=tostring(veh:getInitialNodeCollision(node.cid))
  out.collisionMasks[mask]=(out.collisionMasks[mask] or 0)+1
end
out.cameras=extensions.tech_sensors.getActiveCameraSensors()
out.lidars=extensions.tech_sensors.getActiveLidarSensors()
return jsonEncode(out)
'''
    result=json.loads(b.control.queue_lua_command(lua,response=True))
    (root/'physical_profile.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result),flush=True)
finally:b.control.pause();b.disconnect()
log=Path('/home/ly/beamng-town-retest-user/current/current/beamng.log')
if log.exists():
    selected=[line for line in log.read_text(errors='replace').splitlines()[-1200:]
              if any(k in line for k in ('TechGE','TechVE','wildosBodyCollision','Lua exception','Error','error'))]
    print('\n'.join(line[:500] for line in selected[-15:]),flush=True)
