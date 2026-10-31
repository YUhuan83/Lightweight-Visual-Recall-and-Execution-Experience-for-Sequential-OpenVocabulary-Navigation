"""Verify physical collision masks after a trial without changing physics."""
import json
from pathlib import Path
import sys
from beamngpy import BeamNGpy
root=Path('/home/ly/full_town_memory_20261002')/sys.argv[1]
b=BeamNGpy('127.0.0.1',25252,socket_timeout=8).open(launch=False)
try:
    lua="""
local veh=scenetree.findObject('ego')
local out=extensions.wildosBodyCollision.status(veh:getID())
out.mission=getMissionFilename()
out.collisionMasks={}
for _,node in pairs(core_vehicle_manager.getVehicleData(veh:getID()).vdata.nodes) do
  local mask=tostring(veh:getInitialNodeCollision(node.cid))
  out.collisionMasks[mask]=(out.collisionMasks[mask] or 0)+1
end
out.cameras=extensions.tech_sensors.getActiveCameraSensors()
out.lidars=extensions.tech_sensors.getActiveLidarSensors()
return jsonEncode(out)
"""
    result=json.loads(b.control.queue_lua_command(lua,response=True))
    (root/'collision_profile_after.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result),flush=True)
finally:b.disconnect()

