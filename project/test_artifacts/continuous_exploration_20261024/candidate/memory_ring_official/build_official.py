"""Build an official-assets-only WildOS memory test level.

The level uses the official tech_ground terrain/material files and selected
west_coast_usa house/street assets.  It intentionally does not depend on the
previous procedural memory_ring generator.
"""
from __future__ import annotations

import json
import hashlib
import math
import shutil
import uuid
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "generated"
MOD = OUT / "mod"
LEVEL = "wildos_memory_ring"
PREFIX = f"levels/{LEVEL}/"
CONTENT = Path(r"D:\BEAMNG\BeamNG.tech.v0.39.4.0\content")
WEST_ZIP = CONTENT / "levels" / "west_coast_usa.zip"

CX, CY = 128.0, 128.0
RING = [(48, 88), (72, 55), (120, 45), (170, 55), (204, 86),
        (212, 126), (202, 170), (170, 202), (125, 212), (80, 202),
        (45, 176), (40, 130), (42, 104)]
RING = [(x, y) for x, y in RING]
ROAD_WIDTH = 8.0

HOUSE_MODELS = [
    "house1.dae", "house2.dae", "house3.dae", "house4.dae",
    "house_small1.dae", "house_small2.dae", "house_small3.dae",
    "cabin1.dae", "cabin2.dae",
]


def pid(name: str) -> str:
    return str(uuid.uuid5(uuid.NAMESPACE_URL, f"{LEVEL}/{name}"))


def item(name: str, cls: str, parent: str | None = None, **fields):
    data = {"name": name, "class": cls, "persistentId": pid(name)}
    if parent:
        data["__parent"] = parent
    data.update(fields)
    return data


def write_rows(path: Path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(json.dumps(row, separators=(",", ":")) + "\n" for row in rows), encoding="utf-8")


def write_json(path: Path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")


def copy_entry(zf: zipfile.ZipFile, entry: str):
    destination = MOD / entry
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_bytes(zf.read(entry))


def copy_west_base():
    """Copy west_coast terrain, road, grass and tree material resources."""
    with zipfile.ZipFile(WEST_ZIP) as zf:
        prefixes = (
            "levels/west_coast_usa/art/shapes/trees/",
            "levels/west_coast_usa/art/groundcover/",
            "levels/west_coast_usa/art/road/",
            "levels/west_coast_usa/art/terrains/",
        )
        material_files = (
            "levels/west_coast_usa/art/shapes/main.materials.json",
            "levels/west_coast_usa/art/shapes/trees/main.materials.json",
            "levels/west_coast_usa/art/shapes/trees/trees_conifer/main.materials.json",
            "levels/west_coast_usa/art/shapes/trees/trees_oak/main.materials.json",
            "levels/west_coast_usa/art/shapes/trees/trees_shrub/main.materials.json",
            "levels/west_coast_usa/art/shapes/trees/trees_cypress/main.materials.json",
            "levels/west_coast_usa/art/shapes/trees/trees_eucalyptus/main.materials.json",
            "levels/west_coast_usa/art/shapes/objects/main.materials.json",
            "levels/west_coast_usa/art/shapes/roads/main.materials.json",
            "levels/west_coast_usa/art/groundcover/main.materials.json",
            "levels/west_coast_usa/art/terrains/main.materials.json",
        )
        copied_json = []
        for entry in zf.namelist():
            if entry.startswith(prefixes) or entry in material_files:
                copy_entry(zf, entry)
                if entry.endswith('.json'):
                    copied_json.append(entry)
        for entry in (
            "levels/west_coast_usa/west_coast_usa.ter",
            "levels/west_coast_usa/west_coast_usa.terrain.json",
            "levels/west_coast_usa/info.json",
        ):
            copy_entry(zf, entry)

        # Keep material references at their official absolute paths, but copy
        # the referenced files into this mod so west_coast is self-contained.
        names = set(zf.namelist())
        refs = set()
        for entry in copied_json:
            try:
                text = zf.read(entry).decode('utf-8', 'ignore')
            except Exception:
                continue
            for token in text.replace('"', ' ').split():
                ref = token.lstrip('/').rstrip(',}')
                if ref.startswith('levels/west_coast_usa/'):
                    refs.add(ref)
        for ref in refs:
            if ref in names:
                copy_entry(zf, ref)


def copy_west_assets():
    """Copy selected houses, street objects and their referenced materials."""
    selected = [
        "levels/west_coast_usa/art/shapes/buildings/main.materials.json",
        "levels/west_coast_usa/art/shapes/objects/guardrail2.dae",
        "levels/west_coast_usa/art/shapes/objects/pole_city1.dae",
        "levels/west_coast_usa/art/shapes/buildings/bussign.dae",
    ]
    selected += ["levels/west_coast_usa/art/shapes/buildings/" + name for name in HOUSE_MODELS]
    with zipfile.ZipFile(WEST_ZIP) as zf:
        names = set(zf.namelist())
        material_jsons = []
        for entry in selected:
            if entry in names:
                copy_entry(zf, entry)
                if entry.endswith(".json"):
                    material_jsons.append(zf.read(entry).decode("utf-8", "ignore"))

        # Copy files explicitly referenced by selected west_coast materials.
        refs = set()
        for text in material_jsons:
            for token in text.replace('"', " ").split():
                token = token.lstrip("/")
                if token.startswith("levels/west_coast_usa/"):
                    refs.add(token.rstrip(",}"))
        for ref in refs:
            if ref in names:
                copy_entry(zf, ref)


def nearest_segment_distance(x, y):
    best = 1e9
    for (ax, ay), (bx, by) in zip(RING, RING[1:] + RING[:1]):
        vx, vy = bx - ax, by - ay
        wx, wy = x - ax, y - ay
        denom = vx * vx + vy * vy or 1.0
        t = max(0.0, min(1.0, (wx * vx + wy * vy) / denom))
        best = min(best, math.hypot(x - (ax + t * vx), y - (ay + t * vy)))
    return best


def make_geometry():
    rows = []

    # Official west_coast houses, placed outside the road corridor.
    candidates = []
    for i in range(24):
        angle = i * math.tau / 24.0
        radius = 55.0 if i % 2 else 68.0
        x = CX + math.cos(angle) * radius
        y = CY + math.sin(angle) * radius
        if nearest_segment_distance(x, y) < ROAD_WIDTH / 2 + 15:
            continue
        candidates.append((x, y, angle + math.pi / 2))

    for i, (x, y, yaw) in enumerate(candidates[:18]):
        model = HOUSE_MODELS[i % len(HOUSE_MODELS)]
        c, s = math.cos(yaw), math.sin(yaw)
        rows.append(item(
            f"official_house_{i}", "TSStatic", "Geometry",
            position=[x, y, 0.5],
            rotationMatrix=[c, -s, 0, s, c, 0, 0, 0, 1],
            scale=[1.0 + (i % 3) * 0.08] * 3,
            shapeName=f"/levels/west_coast_usa/art/shapes/buildings/{model}",
            collisionType="Visible Mesh Final",
            useInstanceRenderData=True,
        ))

    # Official guardrails: one object per short interval, tangent-aligned and
    # placed on both sides of the 8m road, never in the center corridor.
    guard_idx = 0
    for ax, ay, bx, by in ((a[0], a[1], b[0], b[1]) for a, b in zip(RING, RING[1:] + RING[:1])):
        length = math.hypot(bx - ax, by - ay)
        yaw = math.atan2(by - ay, bx - ax)
        tx, ty = (bx - ax) / length, (by - ay) / length
        nx, ny = -ty, tx
        count = max(2, int(length / 3.0))
        for j in range(count):
            t = (j + 0.5) / count
            sx, sy = ax + (bx - ax) * t, ay + (by - ay) * t
            for side in (-1, 1):
                px = sx + nx * side * (ROAD_WIDTH / 2 + 0.7)
                py = sy + ny * side * (ROAD_WIDTH / 2 + 0.7)
                c, s = math.cos(yaw), math.sin(yaw)
                rows.append(item(
                    f"official_guardrail_{guard_idx}", "TSStatic", "Geometry",
                    position=[px, py, 0.6], rotationMatrix=[c, -s, 0, s, c, 0, 0, 0, 1],
                    scale=[1, 1, 1],
                    shapeName="/levels/west_coast_usa/art/shapes/objects/guardrail2.dae",
                    collisionType="Visible Mesh Final", useInstanceRenderData=True,
                ))
                guard_idx += 1

    # Official street lights and signs at alternating ring nodes.
    for i, (x, y) in enumerate(RING[::2]):
        angle = math.atan2(y - CY, x - CX)
        c, s = math.cos(angle), math.sin(angle)
        rows.append(item(
            f"official_lamp_{i}", "TSStatic", "Geometry",
            position=[x, y, 0.5], rotationMatrix=[c, -s, 0, s, c, 0, 0, 0, 1],
            shapeName="/levels/west_coast_usa/art/shapes/objects/pole_city1.dae",
            collisionType="Visible Mesh Final", useInstanceRenderData=True,
        ))
    for i, (x, y) in enumerate((RING[0], RING[4], RING[8], RING[12])):
        rows.append(item(
            f"official_bussign_{i}", "TSStatic", "Geometry",
            position=[x + 3, y + 3, 0.5],
            shapeName="/levels/west_coast_usa/art/shapes/buildings/bussign.dae",
            collisionType="Visible Mesh Final", useInstanceRenderData=True,
        ))

    return rows


def build():
    if OUT.exists():
        shutil.rmtree(OUT)
    (MOD / PREFIX / "main" / "MissionGroup" / "Geometry").mkdir(parents=True, exist_ok=True)
    copy_west_base()
    copy_west_assets()

    level_root = MOD / PREFIX
    # Minimal official terrain base from tech_ground.
    write_rows(level_root / "main/items.level.json", [item("MissionGroup", "SimGroup", enabled="1")])
    write_rows(level_root / "main/MissionGroup/items.level.json", [
        item("Level_objects", "SimGroup", "MissionGroup"),
        item("Geometry", "SimGroup", "MissionGroup"),
        item("PlayerDropPoints", "SimGroup", "MissionGroup"),
    ])
    write_rows(level_root / "main/MissionGroup/Level_objects/items.level.json", [
        item("theTerrain", "TerrainBlock", "Level_objects", position=[-1024, -1024, 0],
             materialTextureSet="west_coast_usaTerrainMaterialTextureSet",
             maxHeight=384.640015, terrainFile="/levels/west_coast_usa/west_coast_usa.ter"),
        item("theLevelInfo", "LevelInfo", "Level_objects", gravity=-9.81,
             visibleDistance=16000, canvasClearColor="black"),
    ])
    geometry = make_geometry()
    # Use the official road material names and a small dirt access path to centre.
    def road(name, points, material, width):
        return item(name, "DecalRoad", "Geometry", position=points[0][:3], detail=0.2,
                    distanceFade=[200, 50], drivability=1.0, material=material,
                    nodes=[[x, y, 0.5, width] for x, y in points], renderPriority=1)
    ring_points = RING + [RING[0]]
    geometry.append(road("official_asphalt_ring", ring_points, "road_asphalt_2lane", ROAD_WIDTH))
    geometry.append(road("official_ring_line", ring_points, "m_line_white", 0.20))
    geometry.append(road("official_pond_dirt_access", [(128, 48), (128, 75), (128, 105)], "m_dirt_road", 5.0))
    write_rows(level_root / "main/MissionGroup/Geometry/items.level.json", geometry)
    write_rows(level_root / "main/MissionGroup/PlayerDropPoints/items.level.json", [
        item("spawn_default", "SpawnSphere", "PlayerDropPoints", position=[48, 88, 0.8], radius=2,
             rotationMatrix=[0, 1, 0, -1, 0, 0, 0, 0, 1],
             dataBlock="SpawnSphereMarker", spawnClass="player", spawnDatablock="DefaultPlayerData", enabled="1")
    ])
    write_json(level_root / "info.json", {
        "title": "WildOS Memory Ring Official",
        "description": "Official tech_ground terrain with west_coast street assets.",
        "previews": [], "authors": "WildOS", "size": [200, 200],
        "roads": "official asphalt ring", "supportsTraffic": False, "supportsTimeOfDay": True,
        "defaultSpawnPointName": "spawn_default",
        "spawnPoints": [{"objectname": "spawn_default"}],
    })
    write_json(level_root / "EDITOR_GUIDE.md", {
        "source": "tech_ground + west_coast_usa official assets",
        "road": "official road_asphalt_2lane + m_line_white",
        "assets": "houses/guardrails/lights/signs are west_coast_usa paths",
        "note": "Geometry and official material paths are copied into the mod; edit Geometry items in World Editor.",
    })
    archive = OUT / "wildos_memory_ring_official.zip"
    files = [p for p in (MOD).rglob("*") if p.is_file()]
    with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as zf:
        for p in files:
            zf.write(p, p.relative_to(MOD).as_posix())
    print(json.dumps({"zip": str(archive), "bytes": archive.stat().st_size,
                      "sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
                      "geometry_items": len(geometry)}, indent=2))


if __name__ == "__main__":
    build()
