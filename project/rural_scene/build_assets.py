"""Offline native asset authoring. Never starts BeamNG or installs anything."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import random
import struct
import zlib
import zipfile

ROOT = Path(__file__).resolve().parent
ITALY = Path(r"D:\BEAMNG\BeamNG.tech.v0.39.4.0\content\levels\italy.zip")


def named_json(archive, member):
    info = archive.getinfo(member)
    if info.file_size > 8_000_000:
        raise ValueError(f"Refusing large member: {member}")
    raw = archive.read(member)
    text = raw.decode("utf-8-sig")
    try:
        data = json.loads(text)
        rows = data if isinstance(data, list) else [data]
    except json.JSONDecodeError:
        rows = [json.loads(line) for line in text.splitlines() if line.strip()]
    return rows, hashlib.sha256(raw).hexdigest()


def inspect():
    with zipfile.ZipFile(ITALY) as archive:
        names = archive.namelist()
        print("ROAD MEMBERS", *[n for n in names if "decalroads/asphalt" in n and n.endswith("items.level.json")], sep="\n")
        print("MATERIAL MEMBERS", *[n for n in names if n.endswith("main.materials.json")][:12], sep="\n")
        for member in [n for n in names if "decalroads/asphalt" in n and n.endswith("items.level.json")]:
            rows, digest = named_json(archive, member)
            for row in rows:
                nodes = row.get("nodes", [])
                if nodes and min((p[0]-729)**2+(p[1]-764)**2 for p in nodes) < 140**2:
                    print(json.dumps({"member": member, "sha256": digest, "road": row}))
        member = next(n for n in names if n.endswith("main.materials.json") and archive.getinfo(n).file_size < 100_000)
        rows, digest = named_json(archive, member)
        print("MATERIAL EXAMPLE", member, digest, json.dumps(rows)[:6500])
        for member in [n for n in names if "MissionGroup" in n and n.endswith("items.level.json") and "decalroad" not in n][:30]:
            if archive.getinfo(member).file_size > 1_000_000:
                continue
            rows, digest = named_json(archive, member)
            found = [r for r in rows if r.get("class") == "TSStatic" and r.get("collisionType") == "Visible Mesh"]
            if found:
                print("COLLISION EXAMPLE", member, digest, json.dumps(found[0]))
                break


def formats():
    with zipfile.ZipFile(ITALY) as archive:
        for member in ["levels/italy/art/shapes/misc/main.materials.json", "levels/italy/art/road/main.materials.json"]:
            rows, digest = named_json(archive, member)
            examples = [v for v in rows[0].values() if isinstance(v, dict) and any("diffuseMap" in s for s in v.get("Stages", []))]
            print(member, digest, json.dumps(rows)[:4500])
        for info in archive.infolist():
            if not info.filename.endswith("items.level.json") or info.file_size > 500_000:
                continue
            rows, digest = named_json(archive, info.filename)
            found = [r for r in rows if r.get("class") == "TSStatic" and r.get("collisionType") == "Visible Mesh"]
            if found:
                print(info.filename, digest, json.dumps(found[0]))
                break


def texture(path, color, kind, seed):
    """Original deterministic, tileable mineral/wood/metal RGB PNG, no inputs."""
    rng = random.Random(seed)
    size = 256
    raw = bytearray()
    for y in range(size):
        raw.append(0)
        for x in range(size):
            noise = rng.gauss(0, 7)
            grain = 3*math.sin(x*2*math.pi/64)*math.sin(y*2*math.pi/128)
            if kind == "wood":
                grain += 13*math.sin(x*2*math.pi/16+2*math.sin(y*2*math.pi/256))
            if kind == "concrete" and (x < 2 or y < 2):
                grain -= 23
            raw.extend(max(0, min(255, round(v+noise+grain))) for v in color)
    def chunk(tag, data):
        return struct.pack(">I", len(data))+tag+data+struct.pack(">I", zlib.crc32(tag+data)&0xffffffff)
    path.write_bytes(b"\x89PNG\r\n\x1a\n"+chunk(b"IHDR", struct.pack(">IIBBBBB", size, size, 8, 2, 0, 0, 0))
                     +chunk(b"IDAT", zlib.compress(bytes(raw), 9))+chunk(b"IEND", b""))


def build(italy=ITALY):
    try:
        from .geometry import Route, build_geometry, object_definitions, ART
    except ImportError:
        from geometry import Route, build_geometry, object_definitions, ART
    config = json.loads((ROOT/"course_config.json").read_text(encoding="utf-8"))
    preview = json.loads((ROOT.parent/"beamng_scene/generated/provenance.json").read_text(encoding="utf-8"))
    out = ROOT/"generated"
    out.mkdir(exist_ok=True)
    evidence = {}
    with zipfile.ZipFile(italy) as archive:
        rows, digest = named_json(archive, config["road_member"])
        if digest != preview["evidence"][config["road_member"]]["sha256"]:
            raise ValueError("Italy road metadata differs from verified preview provenance")
        road = next(r for r in rows if r.get("persistentId") == config["road_id"])
        nodes = road["nodes"][config["first_node"]:config["last_node"]+1]
        evidence[config["road_member"]] = digest
        spawn_member = "levels/italy/main/MissionGroup/spawnpoints/items.level.json"
        spawns, digest = named_json(archive, spawn_member)
        spawn = next(s for s in spawns if s.get("name") == preview["spawn"]["name"])
        if spawn["position"] != preview["spawn"]["position"]:
            raise ValueError("Verified spawn changed")
        evidence[spawn_member] = digest
        mat_member = "levels/italy/art/road/main.materials.json"
        mats, digest = named_json(archive, mat_member)
        example = mats[0]["italy_asphalt_overlay_light"]
        assert example["version"] == 1.5 and "baseColorMap" in example["Stages"][0]
        evidence[mat_member] = digest
    route = Route(nodes, config)
    samples = [i*route.length/2000 for i in range(2001)]
    grade = max(abs((route.center(b)[2]-route.center(a)[2])/math.dist(route.center(a)[:2], route.center(b)[:2]))
                for a, b in zip(samples, samples[1:]))
    if grade > config["max_grade"]:
        raise ValueError(f"Unsafe authored grade {grade}")
    meshes, stats = build_geometry(route)
    definitions = object_definitions(route, meshes)
    staging = out/"mod"
    art = staging/ART.strip("/")
    art.mkdir(parents=True, exist_ok=True)
    material_specs = {
        "nz_concrete": ((151, 147, 135), "concrete", "ASPHALT", .94, 0),
        "nz_mortar": ((91, 86, 74), "mineral", "ASPHALT", .98, 0),
        "nz_wood": ((93, 75, 49), "wood", "WOOD", .92, 0),
        "nz_zinc": ((136, 147, 142), "metal", "METAL", .55, .7),
        "nz_iron": ((62, 65, 61), "metal", "METAL", .7, .75),
        "nz_barrel": ((94, 107, 81), "metal", "METAL", .8, .35),
        **{"nz_stone_"+str(i): ((117+i*9, 113+i*8, 101+i*7), "mineral", "ASPHALT", .96, 0) for i in range(4)}
    }
    materials = {}
    package_files = []
    for i, (name, (color, kind, ground, rough, metal)) in enumerate(material_specs.items()):
        path = art/(name+".color.png")
        texture(path, color, kind, config["seed"]+i)
        package_files.append(path)
        # Installed 0.39 JSON serializes editor diffuseMap as baseColorMap in v1.5.
        materials[name] = dict(name=name, mapTo=name, **{"class": "Material"}, version=1.5,
                               Stages=[dict(baseColorMap=ART+path.name, baseColorFactor=[1, 1, 1, 1],
                                            roughnessFactor=rough, metallicFactor=metal), {}, {}, {}],
                               groundType=ground, doubleSided=False, translucentBlendOp="None")
    def write_json(path, data):
        path.write_text(json.dumps(data, indent=2)+"\n", encoding="utf-8")
    write_json(art/"main.materials.json", materials)
    package_files.append(art/"main.materials.json")
    for name, mesh in meshes.items():
        path = art/(name+".dae")
        mesh.write_dae(path)
        package_files.append(path)
    prefab = staging/"art/prefabs/nizhen_rural/course.prefab.json"
    prefab.parent.mkdir(parents=True, exist_ok=True)
    prefab_rows = []
    for definition in definitions:
        item = {k: v for k, v in definition.items() if k not in ("oid", "otype", "pos", "rot_quat")}
        item.update({"class": "TSStatic", "position": definition["pos"], "__parent": "nizhen_rural_group",
                     "rotationMatrix": [1, 0, 0, 0, 1, 0, 0, 0, 1]})
        prefab_rows.append(item)
    prefab_rows.append({"name": "nizhen_rural_group", "class": "SimGroup", "groupPosition": "0 0 0", "noAutoReload": True})
    prefab.write_text("\n".join(json.dumps(r) for r in prefab_rows)+"\n", encoding="utf-8")
    package_files.append(prefab)
    license_path = staging/"art/shapes/nizhen_rural/ASSET_LICENSE.txt"
    license_path.write_text("Original procedural meshes and textures authored for this workspace.\n"
                            "These generated art assets are dedicated to the public domain under CC0 1.0.\n"
                            "https://creativecommons.org/publicdomain/zero/1.0/\n"
                            "No BeamNG textures, meshes, terrain, vehicle assets or secrets are included.\n"
                            "Italy is separately required and remains owned by its rights holders.\n", encoding="ascii")
    package_files.append(license_path)
    s = config["course_spawn_station_m"]
    tangent = route.tangent(s)
    yaw = math.atan2(tangent[1], tangent[0])+math.pi/2
    all_world = []
    for definition in definitions:
        mesh = meshes[Path(definition["shapeName"]).stem]
        bounds = mesh.bounds()
        all_world.extend([[a+b for a, b in zip(definition["pos"], bounds[k])] for k in ("min", "max")])
    road_vertices = [route.world(v) for name, m in meshes.items() if name in ("concrete_entry", "concrete_exit", "cobble_bedding", "cobbles_merged") for v in m.vertices]
    def bounds(points):
        return {"min": [min(p[i] for p in points) for i in range(3)], "max": [max(p[i] for p in points) for i in range(3)]}
    manifest = dict(schema_version=1, level="italy", asset_root=ART, objects=definitions,
                    prefab="/art/prefabs/nizhen_rural/course.prefab.json", origin_world=list(route.origin),
                    route_length_m=route.length, lane_width_m=config["lane_width_m"], max_centerline_grade=grade,
                    road_bounds_world=bounds(road_vertices), all_objects_bounds_world=bounds(all_world),
                    route_nodes_world=nodes, node_stations_m=route.stations,
                    verified_italy_spawn={"name": spawn["name"], "pos": spawn["position"], "rot_quat": preview["rot_quat"],
                                          "note": "Original map spawn, not aligned with course; no generated obstacles placed there."},
                    course_spawn={"pos": route.world(route.point(s, 0, config["vehicle_spawn_lift_m"])),
                                  "rot_quat": [0, 0, math.sin(yaw/2), math.cos(yaw/2)], "station_m": s,
                                  "forward_world": list(tangent), "status": "candidate_not_simulator_verified",
                                  "clear_lane_width_m": config["lane_width_m"], "clear_start_stations_m": [0, 18]},
                    recommended_speed_kph=15, verification=config["verification"], stats=stats)
    write_json(out/"map_objects.json", manifest)
    write_json(out/"route_samples.json", [{"station_m": s, "center_world": route.world(route.point(s)),
                                           "left_world": route.world(route.point(s, config["lane_width_m"]/2)),
                                           "right_world": route.world(route.point(s, -config["lane_width_m"]/2))} for s in samples])
    write_json(out/"asset_provenance.json", {"source": "Italy metadata only, small named ZIP members; no extraction of game art", "members_sha256": evidence,
                                            "road_id": config["road_id"], "source_node_indices_inclusive": [config["first_node"], config["last_node"]],
                                            "material": "Installed v0.39: JSON version 1.5 Stages[].baseColorMap; editor field diffuseMap is not the serialized key.",
                                            "collision": "lua/ge/extensions/editor/resourceChecker/resourceUtil.lua:1710 recognizes Visible Mesh and Visible Mesh Final",
                                            "art": "Entirely original procedural CC0; no third-party textures reused", "runtime_verified": False})
    zip_path = out/"nizhen_rural.zip"
    # Explicit generated-file allowlist, never recursive workspace packaging.
    with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=9) as archive:
        for path in sorted(package_files):
            member = path.relative_to(staging).as_posix()
            if not member.startswith(("art/shapes/nizhen_rural/", "art/prefabs/nizhen_rural/")):
                raise ValueError("Forbidden package member")
            info = zipfile.ZipInfo(member, (2026, 9, 10, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            archive.writestr(info, path.read_bytes())
    report = {"zip": zip_path.name, "zip_sha256": hashlib.sha256(zip_path.read_bytes()).hexdigest(),
              "zip_bytes": zip_path.stat().st_size, "objects": len(definitions), "route_length_m": route.length,
              "max_grade": grade, "meshes": {n: {"triangles": len(m.vertices)//3, "bounds_local": m.bounds()} for n, m in meshes.items()},
              "packaged_files": [p.relative_to(staging).as_posix() for p in package_files]}
    write_json(out/"build_manifest.json", report)
    print(json.dumps({k: v for k, v in report.items() if k not in ("meshes", "packaged_files")}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--italy-zip", type=Path, default=ITALY)
    parser.add_argument("--inspect", action="store_true")
    args = parser.parse_args()
    if args.inspect:
        inspect()
    else:
        build(args.italy_zip)
