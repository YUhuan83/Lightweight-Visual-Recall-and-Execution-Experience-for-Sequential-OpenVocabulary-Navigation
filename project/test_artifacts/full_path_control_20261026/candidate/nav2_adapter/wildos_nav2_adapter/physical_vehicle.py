"""Read-only BeamNG checks for the restored SBR physics, not a motion backend."""
import json
import math


PHYSICS_SNAPSHOT_LUA = """
local v = scenetree.findObject('ego')
assert(v, 'ego vehicle missing')
local d = core_vehicle_manager.getVehicleData(v:getID())
assert(d and d.vdata, 'vehicle data missing')
local out = {vehicleID=v:getID(), mission=getMissionFilename(),
  nodeCount=0, fixedNodeCount=0, massKg=0, beamCount=0,
  bodyCollisionNodeCount=0, wheelCount=0, wheels={},
  -- extensions has an __index autoloader. A presence check must not load
  -- the very physics override that this read-only snapshot is rejecting.
  kinematicExtensionLoaded=rawget(extensions, 'wildosKinematic') ~= nil,
  collisionOverrideLoaded=rawget(extensions, 'wildosBodyCollision') ~= nil,
  configKinematic=d.config.wildosKinematicProfile == true,
  configBodyDisabled=d.config.wildosBodyCollisionDisabled == true}
local nodes = {}
local function collides(cid)
  local mask = tonumber(tostring(v:getInitialNodeCollision(cid)))
  assert(mask ~= nil, 'unreadable engine collision mask')
  return mask ~= 0
end
for _,n in pairs(d.vdata.nodes or {}) do
  nodes[n.cid] = n
  out.nodeCount = out.nodeCount + 1
  if n.fixed then out.fixedNodeCount = out.fixedNodeCount + 1 end
  out.massKg = out.massKg + (tonumber(n.nodeWeight) or 0)
  if (n.wheelID == nil or n.wheelID < 0) and
      collides(n.cid) then
    out.bodyCollisionNodeCount = out.bodyCollisionNodeCount + 1
  end
end
for _,_ in pairs(d.vdata.beams or {}) do out.beamCount = out.beamCount + 1 end
for _,w in pairs(d.vdata.wheels or {}) do
  local tread = {}
  for _,value in pairs(w.treadNodes or {}) do
    local cid = type(value) == 'table' and value.cid or value
    if nodes[cid] then tread[cid] = true end
  end
  if next(tread) == nil then
    for cid,n in pairs(nodes) do
      if n.wheelID == w.wheelID and n.pairedNode ~= nil then tread[cid] = true end
    end
  end
  local count,colliding,fixed = 0,0,0
  for cid,_ in pairs(tread) do
    count=count+1
    if collides(cid) then colliding=colliding+1 end
    if nodes[cid].fixed then fixed=fixed+1 end
  end
  out.wheelCount=out.wheelCount+1
  out.wheels[#out.wheels+1] = {name=w.name or '', treadNodeCount=count,
    collisionNodeCount=colliding, fixedNodeCount=fixed}
end
local p=v:getPosition()
local velocity=v:getVelocity()
out.position={p.x,p.y,p.z}
out.speed=velocity:length()
local sensors=rawget(extensions, 'tech_sensors')
out.cameras=sensors and sensors.getActiveCameraSensors() or {}
out.lidars=sensors and sensors.getActiveLidarSensors() or {}
return jsonEncode(out)
"""


def read_physics_snapshot(beamng):
    return json.loads(beamng.control.queue_lua_command(PHYSICS_SNAPSHOT_LUA, response=True))


def physical_snapshot_errors(snapshot, require_sensors=True):
    """Check observed engine/JBeam state. File config alone cannot pass this."""
    errors = []
    for name in ('kinematicExtensionLoaded', 'collisionOverrideLoaded',
                 'configKinematic', 'configBodyDisabled'):
        if snapshot.get(name) is not False:
            errors.append(name + ': physics override remains or state is missing')
    for name in ('nodeCount', 'beamCount', 'bodyCollisionNodeCount'):
        value = snapshot.get(name)
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            errors.append(name + ': missing physical structures/collision')
    if snapshot.get('fixedNodeCount') != 0:
        errors.append('Vehicle nodes are still fixed')
    mass = snapshot.get('massKg')
    if not isinstance(mass, (int, float)) or not math.isfinite(mass) or mass <= 0:
        errors.append('Vehicle mass is missing or invalid')
    wheels = snapshot.get('wheels', [])
    if (not isinstance(wheels, list) or len(wheels) != 4
            or snapshot.get('wheelCount') != 4):
        errors.append('The SBR must expose all four physical wheels')
    else:
        for wheel in wheels:
            if (not isinstance(wheel, dict) or wheel.get('fixedNodeCount') != 0
                    or wheel.get('treadNodeCount', 0) < 4
                    or wheel.get('collisionNodeCount', 0) < 4):
                errors.append('Wheel lacks dynamic tire collision support')
    if require_sensors and (not snapshot.get('cameras') or not snapshot.get('lidars')):
        errors.append('Camera or LiDAR rig is missing')
    return errors
