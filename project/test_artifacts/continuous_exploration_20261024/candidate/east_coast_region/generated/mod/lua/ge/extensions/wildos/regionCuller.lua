local M = {}
local logTag = "wildosRegionSelector"
local targetLevel = "east_coast_usa"
local configPath = "/mods_data/wildos_east_coast_region/region.json"
local selectionPath = "/settings/wildos_east_coast_region.json"
local config
local selectedRegionId
local pendingFrames = 0

local function currentLevelName()
  local ok, filename = pcall(getMissionFilename)
  if not ok or type(filename) ~= "string" then return "" end
  return filename:match("/levels/([^/]+)/") or ""
end

local function loadConfig()
  if config then return config end
  local ok, value = pcall(jsonReadFile, configPath)
  if ok and type(value) == "table" then config = value end
  return config
end

local function loadSelection()
  local value = loadConfig()
  if not value then return nil end
  local ok, saved = pcall(jsonReadFile, selectionPath)
  local requested = ok and type(saved) == "table" and saved.regionId or value.defaultRegionId
  if not value.regions[requested] then requested = value.defaultRegionId end
  selectedRegionId = requested
  return requested
end

local function selectedRegion()
  local value = loadConfig()
  if not value then return nil end
  local id = selectedRegionId or loadSelection()
  return value.regions[id]
end

local function teleportToSelected()
  if currentLevelName() ~= targetLevel then return false end
  local region = selectedRegion()
  local vehicle = getPlayerVehicle(0)
  if not region or not vehicle then return false end
  local p = region.startPose.position
  local e = region.startPose.rotationEulerDeg
  local rotation = quatFromEuler(math.rad(e[1]), math.rad(e[2]), math.rad(e[3]))
  spawn.safeTeleport(vehicle, vec3(p[1], p[2], p[3]), rotation, nil, nil, nil, true, true)
  log("I", logTag, "Selected region " .. selectedRegionId)
  return true
end

local function arm()
  loadSelection()
  if currentLevelName() == targetLevel then pendingFrames = 45 end
end

local function onUpdate()
  if pendingFrames <= 0 then return end
  pendingFrames = pendingFrames - 1
  if pendingFrames <= 0 and not teleportToSelected() then pendingFrames = 1 end
end

local function selectRegion(regionId, teleportNow)
  local value = loadConfig()
  if not value or not value.regions[regionId] then
    log("E", logTag, "Unknown region: " .. tostring(regionId))
    return false
  end
  selectedRegionId = regionId
  jsonWriteFile(selectionPath, {regionId = regionId}, true)
  if teleportNow ~= false then return teleportToSelected() end
  return true
end

local function getRegions()
  local value = loadConfig()
  return value and value.regions or {}
end

M.onExtensionLoaded = arm
M.onWorldReadyState = function(state) if state == 2 then arm() end end
M.onUpdate = onUpdate
M.selectRegion = selectRegion
M.selectTownIndustrial = function() return selectRegion("eca_town_industrial") end
M.selectSawmillOutdoor = function() return selectRegion("eca_sawmill_outdoor") end
M.getSelectedRegion = function() return selectedRegionId or loadSelection() end
M.getRegions = getRegions
M.teleportToSelected = teleportToSelected
return M
