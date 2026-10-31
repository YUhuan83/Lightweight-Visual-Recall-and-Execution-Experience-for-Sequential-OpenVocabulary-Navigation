"""Rank low-building-density road locations in official East Coast USA."""

from __future__ import annotations

import json
import math
from pathlib import Path
import zipfile


SOURCE = Path(r"D:\BEAMNG\BeamNG.tech.v0.39.4.0\content\levels\east_coast_usa.zip")
PREFIX = "levels/east_coast_usa/"
BUILDING_WORDS = (
    "house", "building", "shop", "store", "warehouse", "church",
    "firestation", "inn", "gasstation", "garage", "sawmill",
)


def load_rows(archive, entry):
    for line in archive.read(entry).decode("utf-8", "ignore").splitlines():
        try:
            yield json.loads(line)
        except json.JSONDecodeError:
            continue


def collect():
    buildings = []
    road_nodes = []
    vegetation = []
    with zipfile.ZipFile(SOURCE) as archive:
        for entry in archive.namelist():
            lower_entry = entry.lower()
            if entry.startswith(PREFIX + "main/") and entry.endswith("items.level.json"):
                for row in load_rows(archive, entry):
                    position = row.get("position")
                    shape = str(row.get("shapeName", "")).lower()
                    parent = str(row.get("__parent", "")).lower()
                    if position and (
                        any(word in shape for word in BUILDING_WORDS)
                        or any(word in parent for word in BUILDING_WORDS)
                    ):
                        buildings.append((float(position[0]), float(position[1]), shape or parent))
                    nodes = row.get("nodes")
                    if nodes and any(part in lower_entry for part in ("/roads/", "/dirt_road/", "/ai_roads/")):
                        road_nodes.extend((float(node[0]), float(node[1]), float(node[2])) for node in nodes)

            if entry.startswith(PREFIX + "forest/eca_bld_") and entry.endswith(".forest4.json"):
                for row in load_rows(archive, entry):
                    position = row.get("pos")
                    object_type = str(row.get("type", "")).lower()
                    if position and any(word in object_type for word in BUILDING_WORDS):
                        buildings.append((float(position[0]), float(position[1]), object_type))
            if entry.startswith(PREFIX + "forest/") and entry.endswith(".forest4.json"):
                item_type = Path(entry).name.lower()
                if any(word in item_type for word in ("tree_", "bush", "fern", "plant")):
                    for row in load_rows(archive, entry):
                        position = row.get("pos")
                        if position:
                            vegetation.append((float(position[0]), float(position[1])))
    return buildings, road_nodes, vegetation


def rank(buildings, road_nodes, vegetation):
    candidates = []
    sampled = []
    for x, y, z in road_nodes:
        if any(math.hypot(x - sx, y - sy) < 45.0 for sx, sy in sampled):
            continue
        sampled.append((x, y))
        distances = sorted(math.hypot(x - bx, y - by) for bx, by, _ in buildings)
        counts = [sum(distance <= radius for distance in distances) for radius in (80, 120, 180)]
        vegetation80 = sum(math.hypot(x - vx, y - vy) <= 80 for vx, vy in vegetation)
        vegetation120 = sum(math.hypot(x - vx, y - vy) <= 120 for vx, vy in vegetation)
        nearest = distances[0] if distances else float("inf")
        score = counts[0] * 20 + counts[1] * 5 + counts[2] + vegetation80 * 0.2 + vegetation120 * 0.03
        candidates.append((score, *counts, vegetation80, vegetation120, nearest, x, y, z))
    return sorted(candidates)


if __name__ == "__main__":
    buildings, road_nodes, vegetation = collect()
    candidates = rank(buildings, road_nodes, vegetation)
    print(f"buildings={len(buildings)} road_nodes={len(road_nodes)} candidates={len(candidates)}")
    print("score n80 n120 n180 veg80 veg120 nearest       x         y       z")
    for candidate in candidates[:80]:
        print("%5.1f %3d %4d %4d %5d %6d %7.1f %9.3f %9.3f %7.3f" % candidate)

    print("\nRecommended 200 x 200 m test box around the best open-road candidate:")
    center = (-450.770, -60.429, 31.572)
    bounds = (center[0] - 100, center[1] - 100, center[0] + 100, center[1] + 100)
    inside_buildings = [item for item in buildings if bounds[0] <= item[0] <= bounds[2] and bounds[1] <= item[1] <= bounds[3]]
    inside_vegetation = [item for item in vegetation if bounds[0] <= item[0] <= bounds[2] and bounds[1] <= item[1] <= bounds[3]]
    print(f"center={center}")
    print(f"bounds=min[{bounds[0]:.3f}, {bounds[1]:.3f}] max[{bounds[2]:.3f}, {bounds[3]:.3f}]")
    print(f"buildings_in_box={len(inside_buildings)} vegetation_instances_in_box={len(inside_vegetation)}")

    print("\nOpen-road candidates within 400 m of the official sawmill spawn:")
    sawmill = (268.829529, -765.437256)
    nearby = [candidate for candidate in candidates
              if math.hypot(candidate[-3] - sawmill[0], candidate[-2] - sawmill[1]) <= 400.0]
    for candidate in nearby[:30]:
        distance = math.hypot(candidate[-3] - sawmill[0], candidate[-2] - sawmill[1])
        print(("dist=%6.1f score=%5.1f buildings=%d/%d/%d vegetation=%d/%d "
               "nearest=%6.1f pos=[%9.3f, %9.3f, %7.3f]") % (
            distance, candidate[0], candidate[1], candidate[2], candidate[3],
            candidate[4], candidate[5], candidate[6], candidate[7], candidate[8], candidate[9]))
