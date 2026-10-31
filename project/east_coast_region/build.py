"""Build the East Coast USA WildOS runtime region-culling overlay."""

from __future__ import annotations

import json
import math
import re
import shutil
from pathlib import Path
import uuid
import zipfile


ROOT = Path(__file__).resolve().parent
OUT = ROOT / "generated"
MOD_ROOT = OUT / "mod"
MOD_ZIP = OUT / "wildos_east_coast_region.zip"
SOURCE_ZIP = Path(r"D:\BEAMNG\BeamNG.tech.v0.39.4.0\content\levels\east_coast_usa.zip")
LEVEL_PREFIX = "levels/east_coast_usa/"


CONFIG = {
    "level": "east_coast_usa",
    "defaultRegionId": "eca_town_industrial",
    "retentionMargin": 10.0,
    "boundaryInclusive": True,
    "regions": {
        "eca_town_industrial": {
            "label": "Town Industrial",
            "startPose": {
                "position": [734.294, -11.946, 52.154],
                "rotationEulerDeg": [0.0, 0.0, -80.042],
            },
            "polygon": [
                [461.856, -3.856],
                [734.294, -11.946],
                [835.645, -285.575],
                [777.200, -461.684],
            ],
        },
        "eca_sawmill_outdoor": {
            "label": "Sawmill Outdoor",
            "startPose": {
                "position": [150.596, -576.816, 41.307],
                "rotationEulerDeg": [0.0, 0.0, 0.0],
            },
            "polygon": [
                [70.596, -496.816],
                [230.596, -496.816],
                [230.596, -656.816],
                [70.596, -656.816],
            ],
        },
    },
}


MOD_SCRIPT = """extensions.load('wildos_regionCuller')\n"""

SPAWN_EXTENSION = r'''local M = {}
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
'''


EXTENSION = r'''local M = {}

local logTag = "wildosRegionCuller"
local configPath = "/mods_data/wildos_east_coast_region/region.json"
local config
local pending = false
local warmupFrames = 0
local objectNames = nil
local objectCursor = 0
local hiddenCount = 0
local keptCount = 0
local errorCount = 0

local alwaysKeepClasses = {
  BeamNGVehicle = true,
  Camera = true,
  CubemapData = true,
  Forest = true,
  ForestItemData = true,
  LevelInfo = true,
  Material = true,
  ScatterSky = true,
  SimGroup = true,
  TerrainBlock = true,
  TimeOfDay = true,
}

local function currentLevelName()
  local ok, filename = pcall(getMissionFilename)
  if not ok or type(filename) ~= "string" then return "" end
  return filename:match("/levels/([^/]+)/") or ""
end

local function loadConfig()
  local ok, data = pcall(jsonReadFile, configPath)
  if not ok or type(data) ~= "table" then
    log("E", logTag, "Cannot read " .. configPath)
    return nil
  end
  return data
end

local function pointInPolygon(x, y, polygon)
  local inside = false
  local j = #polygon
  for i = 1, #polygon do
    local xi, yi = polygon[i][1], polygon[i][2]
    local xj, yj = polygon[j][1], polygon[j][2]
    if ((yi > y) ~= (yj > y)) and
       (x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-12) + xi) then
      inside = not inside
    end
    j = i
  end
  return inside
end

local function distanceToSegment(px, py, ax, ay, bx, by)
  local vx, vy = bx - ax, by - ay
  local wx, wy = px - ax, py - ay
  local denom = vx * vx + vy * vy
  if denom <= 1e-12 then
    local dx, dy = px - ax, py - ay
    return math.sqrt(dx * dx + dy * dy)
  end
  local t = math.max(0, math.min(1, (wx * vx + wy * vy) / denom))
  local dx, dy = px - (ax + t * vx), py - (ay + t * vy)
  return math.sqrt(dx * dx + dy * dy)
end

local function nearBoundary(x, y, margin, polygon)
  local j = #polygon
  for i = 1, #polygon do
    if distanceToSegment(x, y, polygon[j][1], polygon[j][2], polygon[i][1], polygon[i][2]) <= margin then
      return true
    end
    j = i
  end
  return false
end

local function pointKept(x, y)
  return pointInPolygon(x, y, config.polygon) or
    nearBoundary(x, y, config.retentionMargin or 0, config.polygon)
end

local function orientation(ax, ay, bx, by, cx, cy)
  local value = (by - ay) * (cx - bx) - (bx - ax) * (cy - by)
  if math.abs(value) <= 1e-9 then return 0 end
  return value > 0 and 1 or 2
end

local function onSegment(ax, ay, bx, by, cx, cy)
  return bx <= math.max(ax, cx) + 1e-9 and bx >= math.min(ax, cx) - 1e-9 and
    by <= math.max(ay, cy) + 1e-9 and by >= math.min(ay, cy) - 1e-9
end

local function segmentsIntersect(ax, ay, bx, by, cx, cy, dx, dy)
  local o1 = orientation(ax, ay, bx, by, cx, cy)
  local o2 = orientation(ax, ay, bx, by, dx, dy)
  local o3 = orientation(cx, cy, dx, dy, ax, ay)
  local o4 = orientation(cx, cy, dx, dy, bx, by)
  if o1 ~= o2 and o3 ~= o4 then return true end
  return (o1 == 0 and onSegment(ax, ay, cx, cy, bx, by)) or
    (o2 == 0 and onSegment(ax, ay, dx, dy, bx, by)) or
    (o3 == 0 and onSegment(cx, cy, ax, ay, dx, dy)) or
    (o4 == 0 and onSegment(cx, cy, bx, by, dx, dy))
end

local function boxKept(box)
  if not box then return true end
  local margin = config.retentionMargin or 0
  local sourceMin, sourceMax = box.minExtents, box.maxExtents
  local mn = {x = sourceMin.x - margin, y = sourceMin.y - margin}
  local mx = {x = sourceMax.x + margin, y = sourceMax.y + margin}
  local points = {
    {mn.x, mn.y}, {mn.x, mx.y}, {mx.x, mn.y}, {mx.x, mx.y},
    {(mn.x + mx.x) * 0.5, (mn.y + mx.y) * 0.5},
  }
  for _, point in ipairs(points) do
    if pointInPolygon(point[1], point[2], config.polygon) then return true end
  end
  for _, point in ipairs(config.polygon) do
    if point[1] >= mn.x and point[1] <= mx.x and point[2] >= mn.y and point[2] <= mx.y then
      return true
    end
  end
  local rectangle = {
    {mn.x, mn.y}, {mx.x, mn.y}, {mx.x, mx.y}, {mn.x, mx.y},
  }
  local polygonPrevious = config.polygon[#config.polygon]
  for _, polygonPoint in ipairs(config.polygon) do
    local rectanglePrevious = rectangle[#rectangle]
    for _, rectanglePoint in ipairs(rectangle) do
      if segmentsIntersect(
        polygonPrevious[1], polygonPrevious[2], polygonPoint[1], polygonPoint[2],
        rectanglePrevious[1], rectanglePrevious[2], rectanglePoint[1], rectanglePoint[2]
      ) then
        return true
      end
      rectanglePrevious = rectanglePoint
    end
    polygonPrevious = polygonPoint
  end
  return false
end

local function shouldAlwaysKeep(obj)
  local ok, className = pcall(function() return obj:getClassName() end)
  return not ok or alwaysKeepClasses[className] == true
end

local function disableObject(obj)
  pcall(function() obj:setHidden(true) end)
  if config.disableOutsideCollision then
    pcall(function() obj:disableCollision() end)
  end
end

local function processObject(name)
  local obj = scenetree.findObject(name)
  if not obj then
    keptCount = keptCount + 1
    return
  end
  local okClass, className = pcall(function() return obj:getClassName() end)
  if okClass and className == "GroundCover" and config.disableGroundCover then
    disableObject(obj)
    hiddenCount = hiddenCount + 1
    return
  end
  if shouldAlwaysKeep(obj) then
    keptCount = keptCount + 1
    return
  end
  local ok, box = pcall(function() return obj:getWorldBox() end)
  if not ok or boxKept(box) then
    keptCount = keptCount + 1
    return
  end
  disableObject(obj)
  hiddenCount = hiddenCount + 1
end

local function cullForest()
  if not config.removeOutsideForest then return end
  local forest = core_forest and core_forest.getForestObject and core_forest.getForestObject()
  local data = forest and forest:getData()
  if not data then
    log("W", logTag, "ForestData unavailable; forest was not culled")
    return
  end
  local world = Box3F()
  world.minExtents = Point3F(-1e6, -1e6, -1e6)
  world.maxExtents = Point3F(1e6, 1e6, 1e6)
  local removed, kept = 0, 0
  local items = data:getItemsBox(world)
  for _, item in ipairs(items) do
    local ok, box = pcall(function() return item:getWorldBox() end)
    if ok and not boxKept(box) then
      data:removeItem(item)
      removed = removed + 1
    else
      kept = kept + 1
    end
  end
  log("I", logTag, string.format("Forest culled: removed=%d kept=%d", removed, kept))
end

local function beginCull()
  config = loadConfig()
  if not config or currentLevelName() ~= config.level then
    pending = false
    return
  end
  objectNames = scenetree.getAllObjects() or {}
  if config.visibleDistance then
    for _, name in ipairs(scenetree.findClassObjects("LevelInfo") or {}) do
      local levelInfo = scenetree.findObject(name)
      if levelInfo then
        pcall(function() levelInfo:setField("visibleDistance", 0, tostring(config.visibleDistance)) end)
      end
    end
  end
  objectCursor, hiddenCount, keptCount, errorCount = 0, 0, 0, 0
  pending = true
  warmupFrames = 30
  log("I", logTag, string.format("Starting region cull '%s' over %d scene objects", config.regionId, #objectNames))
end

local function finishCull()
  cullForest()
  pending = false
  log("I", logTag, string.format("Region cull complete: hidden=%d kept=%d errors=%d", hiddenCount, keptCount, errorCount))
end

local function onUpdate()
  if not pending then return end
  if warmupFrames > 0 then warmupFrames = warmupFrames - 1; return end
  local batchSize = math.max(1, config.batchSize or 250)
  for _ = 1, batchSize do
    objectCursor = objectCursor + 1
    local name = objectNames[objectCursor]
    if not name then finishCull(); return end
    local ok = pcall(processObject, name)
    if not ok then errorCount = errorCount + 1 end
  end
end

local function onWorldReadyState(state)
  if state == 2 then beginCull() end
end

local function onClientEndMission()
  pending = false
  objectNames = nil
end

M.onExtensionLoaded = beginCull
M.onWorldReadyState = onWorldReadyState
M.onClientEndMission = onClientEndMission
M.onUpdate = onUpdate
M.reapply = beginCull

return M
'''


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8", newline="\n")


def write_json(path: Path, value) -> None:
    write_text(path, json.dumps(value, indent=2) + "\n")


def distance_to_segment(px, py, ax, ay, bx, by):
    vx, vy = bx - ax, by - ay
    wx, wy = px - ax, py - ay
    denominator = vx * vx + vy * vy
    if denominator <= 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / denominator))
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


def point_in_polygon(x, y, points):
    inside = False
    previous = points[-1]
    for point in points:
        ax, ay = point
        bx, by = previous
        if (ay > y) != (by > y) and x < (bx - ax) * (y - ay) / ((by - ay) + 1e-12) + ax:
            inside = not inside
        previous = point
    return inside


def point_kept(x, y, margin=None):
    margin = CONFIG["retentionMargin"] if margin is None else margin
    for region in CONFIG["regions"].values():
        points = region["polygon"]
        if point_in_polygon(x, y, points):
            return True
        if any(distance_to_segment(x, y, *a, *b) <= margin
               for a, b in zip(points, points[1:] + points[:1])):
            return True
    return False


def orientation(a, b, c):
    value = (b[1] - a[1]) * (c[0] - b[0]) - (b[0] - a[0]) * (c[1] - b[1])
    if abs(value) <= 1e-9:
        return 0
    return 1 if value > 0 else 2


def on_segment(a, b, c):
    return (min(a[0], c[0]) - 1e-9 <= b[0] <= max(a[0], c[0]) + 1e-9 and
            min(a[1], c[1]) - 1e-9 <= b[1] <= max(a[1], c[1]) + 1e-9)


def segments_intersect(a, b, c, d):
    o1, o2 = orientation(a, b, c), orientation(a, b, d)
    o3, o4 = orientation(c, d, a), orientation(c, d, b)
    if o1 != o2 and o3 != o4:
        return True
    return ((o1 == 0 and on_segment(a, c, b)) or
            (o2 == 0 and on_segment(a, d, b)) or
            (o3 == 0 and on_segment(c, a, d)) or
            (o4 == 0 and on_segment(c, b, d)))


def polyline_kept(nodes):
    points = [(float(node[0]), float(node[1])) for node in nodes]
    if any(point_kept(*point) for point in points):
        return True
    return any(
        segments_intersect(a, b, c, d)
        for a, b in zip(points, points[1:])
        for region in CONFIG["regions"].values()
        for c, d in zip(region["polygon"], region["polygon"][1:] + region["polygon"][:1])
    )


GLOBAL_CLASSES = {
    "CloudLayer", "CubemapData", "LevelInfo", "ScatterSky", "SimGroup",
    "TerrainBlock", "TimeOfDay",
}


def scene_row_kept(row):
    class_name = row.get("class", "")
    if class_name == "GroundCover":
        return True
    if class_name in GLOBAL_CLASSES:
        return True
    nodes = row.get("nodes")
    if nodes:
        return polyline_kept(nodes)
    position = row.get("position")
    if position and len(position) >= 2:
        margin = 25.0 if class_name in {"TSStatic", "Prefab", "MeshRoad"} else None
        return point_kept(float(position[0]), float(position[1]), margin)
    return True


def filter_scene_files(source, destination):
    stats = {"sceneKept": 0, "sceneRemoved": 0}
    entries = [name for name in source.namelist()
               if name.startswith(LEVEL_PREFIX + "main/") and name.endswith("items.level.json")]
    for entry in entries:
        rows = []
        for line in source.read(entry).decode("utf-8", "ignore").splitlines():
            if not line.strip():
                continue
            row = json.loads(line)
            if scene_row_kept(row):
                rows.append(row)
                stats["sceneKept"] += 1
            else:
                stats["sceneRemoved"] += 1
        write_text(destination / entry, "".join(
            json.dumps(row, separators=(",", ":")) + "\n" for row in rows))
    return stats


def filter_forest_files(source, destination):
    kept = removed = 0
    entries = [name for name in source.namelist()
               if name.startswith(LEVEL_PREFIX + "forest/") and name.endswith(".forest4.json")]
    for entry in entries:
        rows = []
        for line in source.read(entry).decode("utf-8", "ignore").splitlines():
            if not line.strip():
                continue
            row = json.loads(line)
            position = row.get("pos")
            if position and point_kept(float(position[0]), float(position[1]), 12.0):
                rows.append(row)
                kept += 1
            else:
                removed += 1
        write_text(destination / entry, "".join(
            json.dumps(row, separators=(",", ":")) + "\n" for row in rows))
    return {"forestKept": kept, "forestRemoved": removed}


def filter_decals(source, destination):
    entry = LEVEL_PREFIX + "main.decals.json"
    data = json.loads(source.read(entry))
    kept = removed = 0
    for material, instances in list(data.get("instances", {}).items()):
        filtered = []
        for instance in instances:
            if len(instance) > 4 and point_kept(float(instance[3]), float(instance[4]), 12.0):
                filtered.append(instance)
                kept += 1
            else:
                removed += 1
        data["instances"][material] = filtered
    write_json(destination / entry, data)
    return {"decalsKept": kept, "decalsRemoved": removed}


def filter_signals(source, destination):
    entry = LEVEL_PREFIX + "signals.json"
    data = json.loads(source.read(entry))
    instances = data.get("instances", [])
    data["instances"] = [instance for instance in instances
                         if point_kept(float(instance["pos"][0]), float(instance["pos"][1]))]
    write_json(destination / entry, data)
    return {"signalsKept": len(data["instances"]), "signalsRemoved": len(instances) - len(data["instances"])}


def write_spawn_override(source, destination):
    info_entry = LEVEL_PREFIX + "info.json"
    info_text = source.read(info_entry).decode("utf-8", "ignore")
    info = json.loads(re.sub(r",\s*([}\]])", r"\1", info_text))
    info["defaultSpawnPointName"] = "wildos_town_industrial"
    info["spawnPoints"] = [
        {
            "translationId": "WildOS Town Industrial",
            "objectname": "wildos_town_industrial",
            "preview": "spawns_townindustrial.jpg",
        },
        {
            "translationId": "WildOS Sawmill Outdoor",
            "objectname": "wildos_sawmill_outdoor",
            "preview": "spawns_sawmill.jpg",
        },
    ]
    write_json(destination / info_entry, info)

    rows = []
    for region_id, region in CONFIG["regions"].items():
        yaw = math.radians(region["startPose"]["rotationEulerDeg"][2])
        c, s = math.cos(yaw), math.sin(yaw)
        rows.append({
            "name": "wildos_" + region_id.removeprefix("eca_"),
            "class": "SpawnSphere",
            "persistentId": str(uuid.uuid5(uuid.NAMESPACE_URL, "wildos/east_coast_usa/" + region_id)),
            "__parent": "PlayerDropPoints",
            "position": region["startPose"]["position"],
            "rotationMatrix": [c, -s, 0, s, c, 0, 0, 0, 1],
            "dataBlock": "SpawnSphereMarker",
            "enabled": "1",
            "radius": 5,
            "sphereWeight": "1",
        })
    entry = LEVEL_PREFIX + "main/MissionGroup/PlayerDropPoints/items.level.json"
    write_text(destination / entry, "".join(
        json.dumps(row, separators=(",", ":")) + "\n" for row in rows))


def build_precull_overlay(destination):
    with zipfile.ZipFile(SOURCE_ZIP) as source:
        stats = {}
        stats.update(filter_scene_files(source, destination))
        stats.update(filter_forest_files(source, destination))
        stats.update(filter_decals(source, destination))
        stats.update(filter_signals(source, destination))
        write_spawn_override(source, destination)
    return stats


def build() -> Path:
    if OUT.exists():
        shutil.rmtree(OUT)
    stats = build_precull_overlay(MOD_ROOT)
    write_text(MOD_ROOT / "scripts" / "wildos_east_coast_region" / "modScript.lua", MOD_SCRIPT)
    write_text(MOD_ROOT / "lua" / "ge" / "extensions" / "wildos" / "regionCuller.lua", SPAWN_EXTENSION)
    write_json(MOD_ROOT / "mods_data" / "wildos_east_coast_region" / "region.json", CONFIG)
    write_json(MOD_ROOT / "mod_info" / "wildos_east_coast_region" / "info.json", {
        "name": "WildOS East Coast Region Culler",
        "author": "WildOS",
        "version": "0.2.0",
        "description": "Pre-cropped dual-region overlay for East Coast USA WildOS experiments.",
    })

    MOD_ZIP.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(MOD_ZIP, "w", zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(MOD_ROOT.rglob("*")):
            if path.is_file():
                archive.write(path, path.relative_to(MOD_ROOT).as_posix())
    write_json(OUT / "build_stats.json", stats)
    return MOD_ZIP


if __name__ == "__main__":
    print(build())
