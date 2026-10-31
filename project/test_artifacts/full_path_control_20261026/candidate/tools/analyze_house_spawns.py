"""Find American Road house instances nearest to named spawn points."""
from __future__ import annotations

import json
import math
import sys
import zipfile


def vec(row):
    value = row.get("position")
    return [float(v) for v in value] if isinstance(value, list) and len(value) >= 3 else None


spawns = {}
houses = []
with zipfile.ZipFile(sys.argv[1]) as archive:
    for entry in archive.infolist():
        if not entry.filename.endswith("items.level.json"):
            continue
        for line in archive.read(entry).decode("utf-8-sig", errors="replace").splitlines():
            try:
                row = json.loads(line.strip())
            except Exception:
                continue
            position = vec(row)
            if position is None:
                continue
            name = str(row.get("name", ""))
            if row.get("class") == "SpawnSphere" and name.startswith("spawns_"):
                spawns[name] = position
            haystack = " ".join(str(row.get(k, "")) for k in
                                ("name", "internalName", "shapeName", "shapeFile", "fileName")).lower()
            if "house" in haystack and row.get("class") in ("TSStatic", "ForestItemData", "Prefab"):
                houses.append({"name": name, "class": row.get("class"),
                               "position": position, "asset": haystack[:240]})

results = []
for spawn_name, spawn_pos in sorted(spawns.items()):
    nearest = sorted(houses, key=lambda h: math.dist(spawn_pos, h["position"]))[:3]
    results.append({"spawn": spawn_name, "position": spawn_pos,
                    "nearest": [{**h, "distance": math.dist(spawn_pos, h["position"])}
                                for h in nearest]})
print(json.dumps({"spawn_count": len(spawns), "house_count": len(houses),
                  "spawns": results}, indent=2))
