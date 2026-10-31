-- Ideal planar sensor carrier: nodes are fixed, collision is disabled, and
-- ideal mode hides the host's flexbody meshes while retaining its sensors.
-- Motion is integrated from cmd_vel independently in translation and yaw.
local M = {}
local profile = 'all-fixed-v1'
local active, commandAge, linear, angular = nil, 1, 0, 0
local offsetX, offsetY = 0, 0
local carrierModel = 'ideal_point'
local minimumTurningRadius = 8
local droppedDtUpdates, staleCommandUpdates = 0, 0
local lastDtSim = nil
-- ROS angular.z is counterclockwise in the XY navigation plane. In this
-- relative-cluster API, the old positive quaternion increment produced the
-- opposite measured yaw (recorded steady-command gain approximately -1).
-- Convert only the engine rotation; path integration and axle compensation
-- below remain in the ROS convention.
local rosYawToClusterSign = -1
local reports = {}
local cameraIds, lidarIds = {}, {}
local visualMeshCount, hiddenMeshCount = 0, 0
local visualMeshesHidden = false

local function setHostMeshesVisible(id, alpha)
  local bundle = core_vehicle_manager.getVehicleData(id)
  local obj = be:getObjectByID(id)
  if not bundle or not bundle.vdata or not obj then return 0, 0 end
  local seen, total, applied = {}, 0, 0
  for _, flexbody in pairs(bundle.vdata.flexbodies or {}) do
    local name = flexbody.mesh
    if type(name) == 'string' and name ~= '' and not seen[name] then
      seen[name] = true
      total = total + 1
      local ok = pcall(function() obj:setMeshAlpha(alpha, name, false) end)
      if ok then applied = applied + 1 end
    end
  end
  return total, applied
end

local function phase2(_, id, vehicle, _, config)
  if not config or config.wildosKinematicProfile ~= profile then return end
  local count = 0
  for _, node in pairs(vehicle.nodes or {}) do
    node.fixed, node.collision, node.staticCollision, node.selfCollision = true, false, false, false
    count = count + 1
  end
  assert(count > 0, 'No vehicle nodes to fix')
  reports[id] = {nodeCount=count, profile=profile}
end

local function verify(id)
  local bundle = core_vehicle_manager.getVehicleData(id)
  assert(bundle and bundle.config.wildosKinematicProfile == profile, 'Kinematic profile missing')
  local count = 0
  for _, node in pairs(bundle.vdata.nodes or {}) do
    assert(node.fixed == true and node.collision == false and node.staticCollision == false
      and node.selfCollision ~= true, 'A physical node remains enabled')
    count = count + 1
  end
  assert(count > 0, 'No fixed nodes')
  return count
end

local function start(id, x, y, model, radius)
  -- Reloading this extension does not rerun the vehicle's JBeam callback.
  -- Rebuild runtime bookkeeping from the verified live nodes at every start.
  local count = verify(id)
  reports[id] = {nodeCount=count, profile=profile}
  model = model or 'ideal_point'
  assert(model == 'ideal_point' or model == 'ackermann', 'Unknown carrier model')
  active, offsetX, offsetY = id, x, y
  carrierModel = model
  minimumTurningRadius = radius or 0
  if carrierModel == 'ackermann' then
    assert(minimumTurningRadius > 0, 'Ackermann carrier requires positive turning radius')
  end
  commandAge, linear, angular = 1, 0, 0
  droppedDtUpdates, staleCommandUpdates, lastDtSim = 0, 0, nil
  cameraIds = extensions.tech_sensors.getActiveCameraSensors()
  lidarIds = extensions.tech_sensors.getActiveLidarSensors()
  visualMeshCount, hiddenMeshCount = setHostMeshesVisible(id,
    carrierModel == 'ideal_point' and 0 or 1)
  visualMeshesHidden = carrierModel == 'ideal_point'
    and visualMeshCount > 0 and hiddenMeshCount == visualMeshCount
end

local function command(v, w)
  assert(active, 'Kinematic backend not initialized')
  assert(type(v)=='number' and type(w)=='number' and v==v and w==w
    and math.abs(v)<math.huge and math.abs(w)<math.huge, 'Invalid velocity')
  if carrierModel == 'ackermann' then
    w = math.max(-math.abs(v)/minimumTurningRadius,
      math.min(math.abs(v)/minimumTurningRadius, w))
  end
  linear, angular, commandAge = v, w, 0
end

local function stop()
  linear, angular, commandAge = 0, 0, 1
  if active and visualMeshesHidden then
    setHostMeshesVisible(active, 1)
  end
  visualMeshesHidden = false
  visualMeshCount, hiddenMeshCount = 0, 0
end

local function update(dtReal, dtSim)
  if not active then return end
  commandAge = commandAge + math.max(0, dtReal)
  if commandAge > .6 then
    staleCommandUpdates = staleCommandUpdates + 1
    return
  end
  lastDtSim = dtSim
  if dtSim <= 0 or dtSim > .25 then
    droppedDtUpdates = droppedDtUpdates + 1
    return
  end
  local veh = scenetree.findObjectById(active)
  if not veh then stop(); active=nil; return end
  if linear == 0 and angular == 0 then return end
  local pos, dir = veh:getPosition(), veh:getDirectionVector()
  local heading = math.atan2(dir.y, dir.x)
  local turn = angular*dtSim
  local dx, dy
  if math.abs(angular) > 0.000001 then
    dx = linear/angular*(math.sin(heading+turn)-math.sin(heading))
    dy = linear/angular*(math.cos(heading)-math.cos(heading+turn))
  else
    dx, dy = linear*dtSim*math.cos(heading), linear*dtSim*math.sin(heading)
  end
  -- Input speed is at the navigation origin, while the engine pose uses the
  -- ref node. Match the adapter's 3D forward/left body axes even when the
  -- sensor host retains pitch or roll; planar offsets alone drift on rotation.
  local up = veh:getDirectionVectorUp()
  local fn = math.sqrt(dir.x*dir.x+dir.y*dir.y+dir.z*dir.z)
  assert(fn > 0.000001, 'Invalid carrier forward axis')
  local fx, fy, fz = dir.x/fn, dir.y/fn, dir.z/fn
  local lx, ly, lz = up.y*fz-up.z*fy, up.z*fx-up.x*fz, up.x*fy-up.y*fx
  local ln = math.sqrt(lx*lx+ly*ly+lz*lz)
  assert(ln > 0.000001, 'Invalid carrier left axis')
  local ox, oy = offsetX*fx+offsetY*lx/ln, offsetX*fy+offsetY*ly/ln
  local nextOx = ox*math.cos(turn)-oy*math.sin(turn)
  local nextOy = ox*math.sin(turn)+oy*math.cos(turn)
  pos.x = pos.x + dx + ox-nextOx
  pos.y = pos.y + dy + oy-nextOy
  local current = quat(veh:getClusterRotationSlow(veh:getRefNodeId()))
  -- BeamNG mathlib applies quaternions to vectors with -w: Q(a*b)=Q(b)Q(a).
  -- Right-multiply to apply world-Z yaw after the current tilted orientation.
  -- Left-multiplication would rotate around the host's local Z instead.
  local target = current*quatFromAxisAngle(vec3(0,0,1), rosYawToClusterSign*turn)
  local delta = current:inversed()*target
  veh:setClusterPosRelRot(veh:getRefNodeId(), pos.x,pos.y,pos.z,delta.x,delta.y,delta.z,delta.w)
end

local function status()
  assert(active, 'Kinematic backend not initialized')
  local veh = scenetree.findObjectById(active)
  assert(veh, 'Vehicle disappeared')
  return {position=veh:getPosition():toTable(), direction=veh:getDirectionVector():toTable(),
    up=veh:getDirectionVectorUp():toTable(), commandAge=commandAge,
    nodeCount=reports[active].nodeCount, profile=profile, physicsBypassed=true,
    mission=getMissionFilename(), carrierModel=carrierModel,
    minimumTurningRadius=minimumTurningRadius,
    rosYawToClusterSign=rosYawToClusterSign,
    rotationConvention='world-z-passive-v2',
    visualMeshesHidden=visualMeshesHidden, visualMeshCount=visualMeshCount,
    hiddenMeshCount=hiddenMeshCount,
    droppedDtUpdates=droppedDtUpdates, staleCommandUpdates=staleCommandUpdates,
    lastDtSim=lastDtSim, cameras=cameraIds, lidars=lidarIds}
end

M.onJbeamLoadingPhase2, M.onUpdate = phase2, update
M.start, M.command, M.stop, M.status = start, command, stop, status
return M
