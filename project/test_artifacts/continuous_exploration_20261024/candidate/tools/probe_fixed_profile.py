import json
from beamngpy import BeamNGpy
b = BeamNGpy('127.0.0.1',25252,socket_timeout=8).open(launch=False)
try:
    print(b.control.queue_lua_command("""
local v=scenetree.findObject('ego')
local d=core_vehicle_manager.getVehicleData(v:getID())
local out={profile=d.config.wildosKinematicProfile,nodes={},masks={}}
for _,n in pairs(d.vdata.nodes) do
 local mask=v:getInitialNodeCollision(n.cid)
 out.masks[tostring(mask)]=(out.masks[tostring(mask)] or 0)+1
 if #out.nodes<4 then table.insert(out.nodes,{cid=n.cid,fixed=n.fixed,collision=n.collision,
   staticCollision=n.staticCollision,selfCollision=n.selfCollision}) end
end
return jsonEncode(out)
""",response=True))
finally:
    b.disconnect()
