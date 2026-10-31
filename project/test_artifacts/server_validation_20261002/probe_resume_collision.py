"""Resume paused startup and report the live Town sensor/collision state."""
import json
from pathlib import Path
from beamngpy import BeamNGpy

root=Path('/home/ly/full_town_memory_20261002/body_collision_full')
root.mkdir(exist_ok=True)
b=BeamNGpy('127.0.0.1',25252,socket_timeout=8).open(launch=False)
try:
    command="""
local veh=scenetree.findObject('ego')
assert(veh, 'ego missing')
local bundle=core_vehicle_manager.getVehicleData(veh:getID())
local masks={}
for _,node in pairs(bundle.vdata.nodes) do
  local mask=tostring(veh:getInitialNodeCollision(node.cid))
  masks[mask]=(masks[mask] or 0)+1
end
local profile=extensions.wildosBodyCollision and extensions.wildosBodyCollision.status(veh:getID()) or {}
return jsonEncode({mission=getMissionFilename(),config=bundle.config,
  collisionMasks=masks,profile=profile,
  cameras=extensions.tech_sensors.getActiveCameraSensors(),
  lidars=extensions.tech_sensors.getActiveLidarSensors(),
  physicsEnabled=be:getEnabled()})
"""
    result=json.loads(b.control.queue_lua_command(command,response=True))
    result['gamestate']=b.control.get_gamestate()
    b.control.resume()
    result['resume_requested']=True
    (root/'startup_probe.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result),flush=True)
finally:
    b.disconnect()

