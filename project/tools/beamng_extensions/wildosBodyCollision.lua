-- BeamNG.tech 0.39.x GE extension for selective body collision suppression.
-- Load before spawning/reloading the vehicle. The mode belongs in the vehicle
-- config so that the JBeam loader's config cache separates the two profiles.
-- This changes no node positions, masses, beams, selfCollision, or triangles.

local M = {}
local reports = {}
local lastVehicleID
local logTag = "wildosBodyCollision"

local function copy(value)
  if type(value) ~= "table" then return value end
  local result = {}
  for key, item in pairs(value) do result[key] = copy(item) end
  return result
end

local function validID(value)
  return type(value) == "number" and value >= 0 and value == math.floor(value)
end

local function groundCollisionEnabled(node)
  local collision = node.collision
  if collision == nil then collision = true end
  local staticCollision = node.staticCollision
  if staticCollision == nil then staticCollision = collision end
  return collision == true and staticCollision == true
end

-- Resolve both pre-optimization node tables and post-optimization numeric cids.
-- Validate the complete protection set before changing any node.
local function inspect(vehicle)
  local nodes, protected, treadProtected, wheelIDs = {}, {}, {}, {}
  local result = {
    totalNodeCount = 0, wheelCount = 0, groundedTireWheelCount = 0,
    protectedWheelNodeCount = 0, protectedTreadNodeCount = 0,
    bodyNodeCount = 0, bodyCollisionDisabledCount = 0, wheels = {}
  }
  for _, node in pairs(vehicle.nodes or {}) do
    if type(node) ~= "table" or not validID(node.cid) or nodes[node.cid] then
      return nil, "invalid or duplicate node cid"
    end
    nodes[node.cid] = node
    result.totalNodeCount = result.totalNodeCount + 1
  end
  if result.totalNodeCount == 0 then return nil, "no nodes" end

  local function resolve(value)
    local cid = type(value) == "table" and value.cid or value
    if not validID(cid) or not nodes[cid] then return nil end
    return cid
  end
  local function protect(value)
    local cid = resolve(value)
    if not cid then return false end
    protected[cid] = true
    return true
  end
  for cid, node in pairs(nodes) do
    if validID(node.wheelID) then protected[cid] = true end
  end

  for _, wheel in pairs(vehicle.wheels or {}) do
    if type(wheel) ~= "table" then return nil, "invalid wheel entry" end
    if not validID(wheel.wheelID) or wheelIDs[wheel.wheelID] then
      return nil, "invalid or duplicate wheel id"
    end
    wheelIDs[wheel.wheelID] = true
    result.wheelCount = result.wheelCount + 1
    local tread, hubCount = {}, 0
    for _, value in pairs(wheel.nodes or {}) do
      if not protect(value) then return nil, "invalid wheel hub node reference" end
      hubCount = hubCount + 1
    end
    for _, value in pairs(wheel.treadNodes or {}) do
      local cid = resolve(value)
      if not cid then return nil, "invalid tire tread node reference" end
      protected[cid], tread[cid], treadProtected[cid] = true, true, true
    end
    -- Older wheel implementations expose paired tread nodes rather than an
    -- explicit treadNodes list. Never count hub/axle nodes as tire contact.
    if next(tread) == nil and validID(wheel.wheelID) then
      for cid, node in pairs(nodes) do
        if node.wheelID == wheel.wheelID and node.pairedNode ~= nil then
          protected[cid], tread[cid], treadProtected[cid] = true, true, true
        end
      end
    end
    for _, field in ipairs({"node1", "node2"}) do
      if not protect(wheel[field]) then return nil, "invalid wheel axle node reference" end
    end
    local treadCount, groundCount = 0, 0
    for cid in pairs(tread) do
      treadCount = treadCount + 1
      if groundCollisionEnabled(nodes[cid]) then groundCount = groundCount + 1 end
    end
    if hubCount > 0 and groundCount >= 4 then
      result.groundedTireWheelCount = result.groundedTireWheelCount + 1
    end
    result.wheels[#result.wheels + 1] = {
      name = wheel.name or "", wheelID = wheel.wheelID,
      hubNodeCount = hubCount, treadNodeCount = treadCount,
      groundCollisionTreadNodeCount = groundCount
    }
  end

  -- Pair references can be present on generated tire nodes outside a wheel's
  -- explicit lists. Follow them to closure before classifying body nodes.
  local changed = true
  while changed do
    changed = false
    for cid in pairs(protected) do
      local node = nodes[cid]
      for _, field in ipairs({"pairedNode", "pairedNode2"}) do
        if node[field] ~= nil then
          local pair = resolve(node[field])
          if not pair then return nil, "invalid paired tire node reference" end
          if not protected[pair] then protected[pair], changed = true, true end
          treadProtected[cid], treadProtected[pair] = true, true
        end
      end
    end
  end
  for cid, node in pairs(nodes) do
    if protected[cid] then
      result.protectedWheelNodeCount = result.protectedWheelNodeCount + 1
    else
      result.bodyNodeCount = result.bodyNodeCount + 1
      if node.collision == false and node.staticCollision == false then
        result.bodyCollisionDisabledCount = result.bodyCollisionDisabledCount + 1
      end
    end
    if treadProtected[cid] then result.protectedTreadNodeCount = result.protectedTreadNodeCount + 1 end
  end
  return result, nil, nodes, protected
end

local function onJbeamLoadingPhase2(_, objID, vehicle, _, config)
  lastVehicleID = objID
  local requested = type(config) == "table" and config.wildosBodyCollisionDisabled == true
  local report = {
    vehicleID = objID, configMode = requested, applied = false,
    phase = requested and "validating" or "disabled", source = "onJbeamLoadingPhase2",
    profileVersion = type(config) == "table" and config.wildosCollisionProfileVersion or nil,
    selfCollisionPreserved = true, trianglesPreserved = true
  }
  reports[objID] = report
  if not requested then return end
  if type(vehicle) ~= "table" then
    report.phase, report.reason = "skipped", "missing vehicle data"
    log("E", logTag, "Body collision unchanged: " .. report.reason)
    return
  end
  local counts, reason, nodes, protected = inspect(vehicle)
  if counts then
    for key, value in pairs(counts) do report[key] = value end
    if counts.wheelCount < 4 or counts.groundedTireWheelCount < 4 then
      reason = "require at least four wheels with four preserved ground-contact tread nodes each"
    elseif counts.bodyNodeCount == 0 then
      reason = "no body nodes found outside the protected wheel set"
    end
  end
  if reason then
    report.phase, report.reason = "skipped", reason
    log("E", logTag, "Vehicle " .. tostring(objID) .. ": body collision unchanged: " .. reason)
    return
  end
  local changedCount = 0
  for cid, node in pairs(nodes) do
    if not protected[cid] then
      if node.collision ~= false or node.staticCollision ~= false then changedCount = changedCount + 1 end
      node.collision, node.staticCollision = false, false
    end
  end
  report.bodyNodesChangedCount = changedCount
  report.bodyCollisionDisabledCount = report.bodyNodeCount
  report.phase, report.applied = "applied", true
  log("I", logTag, "Vehicle " .. tostring(objID) .. ": disabled " .. tostring(report.bodyNodeCount)
    .. " body nodes; preserved " .. tostring(report.protectedWheelNodeCount) .. " wheel nodes and "
    .. tostring(report.groundedTireWheelCount) .. " tires with ground contact")
end

-- Querying is explicit; there are no per-frame hooks. Recount live GE data so
-- status also verifies cached JBeam spawns that do not execute the phase hook.
local function status(objID)
  objID = objID or lastVehicleID
  local result = copy(reports[objID] or {vehicleID = objID, applied = false, phase = "unknown"})
  if core_vehicle_manager and core_vehicle_manager.getVehicleData and objID ~= nil then
    local bundle = core_vehicle_manager.getVehicleData(objID)
    if bundle and type(bundle.vdata) == "table" then
      result.configMode = type(bundle.config) == "table" and bundle.config.wildosBodyCollisionDisabled == true
      result.profileVersion = type(bundle.config) == "table" and bundle.config.wildosCollisionProfileVersion or nil
      local counts, reason = inspect(bundle.vdata)
      if counts then
        result.liveCounts = counts
        result.liveVerified = result.configMode and counts.bodyNodeCount > 0
          and counts.bodyCollisionDisabledCount == counts.bodyNodeCount
          and counts.wheelCount >= 4 and counts.groundedTireWheelCount >= 4
      else
        result.liveVerified, result.liveQueryError = false, reason
      end
    end
  end
  return result
end

M.onJbeamLoadingPhase2 = onJbeamLoadingPhase2
M.status = status
return M
