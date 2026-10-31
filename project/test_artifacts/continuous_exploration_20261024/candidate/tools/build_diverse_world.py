"""Deterministic, offline 80 m preview. Never writes the original worlds."""
import json
import math
import random
import xml.etree.ElementTree as ET
from pathlib import Path

from build_world import node, rover, segment_distance, write_xml

ROOT = Path(__file__).resolve().parents[1]
MODELS = ROOT / "assets/models"
WORLD = ROOT / "assets/worlds/diverse_preview.sdf"
MANIFEST = ROOT / "assets/diverse_manifest.json"
STEP = 0.5
N = 161
SEED = 80909
DIRT = [(12 * math.sin(y / 16), y) for y in (i * STEP for i in range(81))]
COLORS = [(0.53, 0.52, 0.48), (0.40, 0.28, 0.15), (0.27, 0.36, 0.14),
          (0.36, 0.40, 0.19), (0.43, 0.38, 0.27), (0.25, 0.21, 0.15)]
SURFACES = ["concrete", "dirt", "meadow", "dry_grass", "exposed_soil", "pothole"]
POTHOLES = [(21, 0.75, 1.25, 0.85, 0.18), (-25, -0.65, 1.5, 0.9, 0.22)]


def smooth(t):
    t = max(0.0, min(1.0, t))
    return t * t * (3 - 2 * t)


def distances(x, y):
    # Only adjacent segments are needed because the dirt road is monotonic in Y.
    j = max(0, min(79, int(y / STEP)))
    dirt = min(segment_distance(x, y, DIRT[i], DIRT[i + 1])
               for i in range(max(0, j - 10), min(80, j + 11)))
    return abs(y), dirt


def height(x, y):
    concrete, dirt = distances(x, y)
    hills = 4.8 * math.exp(-((x - 22) ** 2 / 270 + (y - 26) ** 2 / 400))
    hills += 2.8 * math.exp(-((x + 24) ** 2 / 220 + (y - 22) ** 2 / 280))
    hills -= 1.1 * math.exp(-((x + 20) ** 2 / 160 + (y + 23) ** 2 / 180))
    hills += 0.65 * math.sin(x / 14) * math.sin(y / 17)
    # Smooth, level concrete corridor and feathered shoulders, not stacked roads.
    z = hills * smooth((concrete - 2.6) / 7)
    rough = 0.13 * math.sin(x * 1.4) * math.sin(y * 1.1) + 0.07 * math.sin(x * 2.3 + y)
    z += rough * smooth((min(concrete - 2.6, dirt - 2) - 0.5) / 3)
    for px, py, rx, ry, depth in POTHOLES:
        r2 = ((x - px) / rx) ** 2 + ((y - py) / ry) ** 2
        if r2 < 1:
            z -= depth * (1 - r2) ** 2
    # Two actual depressed tracks, restricted to a distant section of the dirt road.
    envelope = smooth((y - 15) / 4) * smooth((34 - y) / 4)
    offset = x - 12 * math.sin(y / 16)
    for track in (-0.65, 0.65):
        z -= 0.105 * math.exp(-((offset - track) / 0.38) ** 2) * envelope
    return z


def surface(x, y):
    concrete, dirt = distances(x, y)
    if concrete < 2.5:
        if any(((x - px) / rx) ** 2 + ((y - py) / ry) ** 2 < 0.8
               for px, py, rx, ry, _ in POTHOLES):
            return 5
        return 0
    if dirt < 1.8:
        return 1
    patch = math.sin(x / 7) + math.cos(y / 9) + 0.6 * math.sin((x + y) / 4)
    return 4 if patch > 1.5 else (3 if patch > 0.1 else 2)


def write_mesh(path, vertices, buckets, colors, names, uv=None):
    """Indexed triangles with area-weighted normals and explicit material binding."""
    normals = [[0.0, 0.0, 0.0] for _ in vertices]
    for indices in buckets:
        for t in range(0, len(indices), 3):
            a, b, c = indices[t:t + 3]
            u = [vertices[b][k] - vertices[a][k] for k in range(3)]
            v = [vertices[c][k] - vertices[a][k] for k in range(3)]
            cross = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
            for index in (a, b, c):
                for k in range(3):
                    normals[index][k] += cross[k]
    for normal in normals:
        length = math.sqrt(sum(v*v for v in normal))
        if length < 1e-12:
            raise ValueError("Unused vertex or cancelling normals")
        normal[:] = [v / length for v in normal]
    dae = ET.Element("COLLADA", xmlns="http://www.collada.org/2005/11/COLLADASchema", version="1.4.1")
    asset = node(dae, "asset")
    for tag in ("created", "modified"):
        node(asset, tag, "2026-09-09T00:00:00Z")
    node(asset, "unit", name="meter", meter="1")
    node(asset, "up_axis", "Z_UP")
    effects, materials = node(dae, "library_effects"), node(dae, "library_materials")
    for k, color in enumerate(colors):
        effect = node(effects, "effect", id=f"e{k}")
        lambert = node(node(node(effect, "profile_COMMON"), "technique", sid="common"), "lambert")
        node(node(lambert, "diffuse"), "color", " ".join(map(str, (*color, 1))))
        node(node(materials, "material", id=f"m{k}", name=names[k]), "instance_effect", url=f"#e{k}")
    mesh = node(node(node(dae, "library_geometries"), "geometry", id="geometry"), "mesh")
    for name, values, axes in [("positions", vertices, "XYZ"), ("normals", normals, "XYZ")] + (
            [("uv", uv, "ST")] if uv is not None else []):
        source = node(mesh, "source", id=name)
        node(source, "float_array", " ".join(f"{v:.7f}" for row in values for v in row),
             id=f"{name}-array", count=len(values)*len(axes))
        accessor = node(node(source, "technique_common"), "accessor", source=f"#{name}-array",
                        count=len(values), stride=len(axes))
        for axis in axes:
            node(accessor, "param", name=axis, type="float")
    node(node(mesh, "vertices", id="verts"), "input", semantic="POSITION", source="#positions")
    for k, indices in enumerate(buckets):
        if not indices:
            continue
        tri = node(mesh, "triangles", material=f"mat{k}", count=len(indices)//3)
        node(tri, "input", semantic="VERTEX", source="#verts", offset=0)
        node(tri, "input", semantic="NORMAL", source="#normals", offset=1)
        if uv is not None:
            node(tri, "input", semantic="TEXCOORD", source="#uv", offset=2, set=0)
        node(tri, "p", " ".join(" ".join([str(i)] * (3 if uv is not None else 2)) for i in indices))
    scene = node(node(dae, "library_visual_scenes"), "visual_scene", id="Scene")
    common = node(node(node(node(scene, "node", id="object"), "instance_geometry", url="#geometry"),
                       "bind_material"), "technique_common")
    for k in range(len(colors)):
        node(common, "instance_material", symbol=f"mat{k}", target=f"#m{k}")
    node(node(dae, "scene"), "instance_visual_scene", url="#Scene")
    write_xml(path, dae)


def mesh_model(name, vertices, buckets, colors, names, uv=None, trunk_height=None):
    folder = MODELS / name
    write_mesh(folder / "visual.dae", vertices, buckets, colors, names, uv)
    sdf = ET.Element("sdf", version="1.8")
    model = node(sdf, "model", name=name)
    node(model, "static", "true")
    link = node(model, "link", name="body")
    for kind in (("visual", "collision") if trunk_height is None else ("visual",)):
        part = node(link, kind, name=kind)
        if kind == "visual":
            node(part, "cast_shadows", "false")
        node(node(node(part, "geometry"), "mesh"), "uri", f"model://{name}/visual.dae")
    if trunk_height is not None:
        collision = node(link, "collision", name="trunk_proxy")
        node(collision, "pose", f"0 0 {trunk_height/2} 0 0 0")
        cylinder = node(node(collision, "geometry"), "cylinder")
        node(cylinder, "radius", 0.3)
        node(cylinder, "length", trunk_height)
    write_xml(folder / "model.sdf", sdf)
    config = ET.Element("model")
    node(config, "name", name)
    node(config, "version", "1.0")
    node(config, "sdf", "model.sdf", version="1.8")
    node(config, "description", "Original procedural preview geometry; not a scanned asset.")
    write_xml(folder / "model.config", config)


def tree(variant):
    rng = random.Random(SEED + variant)
    vertices, buckets = [], [[], [], [], []]

    def rings(centers, radii, material, irregular=False):
        # Single pole vertices avoid degenerate triangles at the closed tips.
        start = len(vertices)
        vertices.append(centers[0])
        sides = 11
        for center, radius in zip(centers[1:-1], radii[1:-1]):
            for i in range(sides):
                angle = 2 * math.pi * i / sides
                jitter = rng.uniform(0.77, 1.22) if irregular else 1
                vertices.append((center[0]+radius[0]*math.cos(angle)*jitter,
                                 center[1]+radius[1]*math.sin(angle)*jitter,
                                 center[2] + (rng.uniform(-0.14, 0.14) if irregular else 0)))
        end = len(vertices)
        vertices.append(centers[-1])
        faces = buckets[material]
        count = len(centers) - 2
        for i in range(sides):
            nxt = (i + 1) % sides
            faces.extend((start, start+1+nxt, start+1+i))
            for j in range(count - 1):
                a, b = start+1+j*sides+i, start+1+j*sides+nxt
                faces.extend((a, b, b+sides, a, b+sides, a+sides))
            faces.extend((end, start+1+(count-1)*sides+i, start+1+(count-1)*sides+nxt))

    h = rng.uniform(4.7, 6.6)
    lean = rng.uniform(-0.35, 0.35)
    rings([(0,0,-0.12), (0,0,0), (lean*.4,0,h*.4), (lean,.12,h*.72), (lean,.12,h*.8)],
          [(0,0), (.25,.23), (.18,.16), (.065,.06), (0,0)], 0)
    for i in range(9):
        angle = i * 2.399 + rng.uniform(-0.2, 0.2)
        reach = rng.uniform(.7, 1.65)
        x, y, z = reach*math.cos(angle), reach*math.sin(angle), h * rng.uniform(.58, .92)
        rings([(lean*.4,0,h*.38), (lean*.4,0,h*.4), (x*.55,y*.55,z-.5), (x,y,z), (x,y,z+.05)],
              [(0,0), (.09,.09), (.065,.065), (.025,.025), (0,0)], 0)
        rx, ry, rz = rng.uniform(.95,1.6), rng.uniform(.85,1.45), rng.uniform(1,1.6)
        levels = [-1, -.8, -.4, 0, .4, .8, 1]
        rings([(x + .18*t, y + .1*t*t, z+rz*t) for t in levels],
              [(rx*math.sqrt(max(0,1-t*t)), ry*math.sqrt(max(0,1-t*t))) for t in levels],
              1+i%3, True)
    mesh_model(f"diverse_tree_{variant}", vertices, buckets,
               [(.25,.16,.08), (.16,.28,.065), (.24,.36,.09), (.31,.39,.12)],
               ["bark", "leaf_dark", "leaf_mid", "leaf_light"], trunk_height=h*.65)


def build():
    vertices = [(i*STEP-40, j*STEP-40, height(i*STEP-40, j*STEP-40))
                for j in range(N) for i in range(N)]
    buckets = [[] for _ in COLORS]
    for j in range(N-1):
        for i in range(N-1):
            a = j*N+i
            k = surface(i*STEP-40+STEP/2, j*STEP-40+STEP/2)
            buckets[k].extend((a, a+1, a+N+1, a, a+N+1, a+N))
    mesh_model("diverse_terrain", vertices, buckets, COLORS, SURFACES,
               uv=[(x/4, y/4) for x, y, _ in vertices])
    for i in range(8):
        tree(i)
    sdf = ET.Element("sdf", version="1.8")
    world = node(sdf, "world", name="diverse_preview")
    physics = node(world, "physics", name="physics", type="ignored")
    node(physics, "max_step_size", .002)
    node(physics, "real_time_factor", 1)
    node(world, "gravity", "0 0 -9.81")
    for short, classname in (("physics", "Physics"), ("user-commands", "UserCommands"),
                             ("scene-broadcaster", "SceneBroadcaster")):
        node(world, "plugin", filename=f"ignition-gazebo-{short}-system",
             name=f"ignition::gazebo::systems::{classname}")
    scene = node(world, "scene")
    node(scene, "ambient", ".65 .65 .65 1")
    node(scene, "background", ".64 .77 .87 1")
    node(scene, "shadows", "false")
    light = node(world, "light", name="sun", type="directional")
    node(light, "direction", "-.4 -.3 -1")
    node(light, "diffuse", ".85 .82 .75 1")
    node(light, "specular", ".05 .05 .05 1")
    node(light, "cast_shadows", "false")
    node(node(world, "include"), "uri", "model://diverse_terrain")
    placements = []

    def include(name, model, x, y, yaw):
        inc = node(world, "include")
        node(inc, "uri", f"model://{model}")
        node(inc, "name", name)
        z = height(x, y)
        node(inc, "pose", f"{x:.5f} {y:.5f} {z:.5f} 0 0 {yaw:.5f}")
        placements.append(dict(name=name, model=model, position=[x,y,z]))

    rng = random.Random(SEED)
    attempts = 0
    while len(placements) < 44:
        attempts += 1
        if attempts > 10000:
            raise RuntimeError("Unable to place trees")
        x, y = rng.uniform(-35,35), rng.uniform(-35,35)
        dc, dd = distances(x, y)
        if dc < 7 or dd < 6 or math.hypot(x,y) < 10:
            continue
        if any(math.hypot(x-p['position'][0], y-p['position'][1]) < 5.5 for p in placements):
            continue
        i = len(placements)
        include(f"broadleaf_{i:02d}", f"diverse_tree_{i%8}", x, y, rng.uniform(-math.pi,math.pi))
    for i, (asset, x, y) in enumerate([
            ("rock_moss_set_01", 25, 18), ("rock_moss_set_01", -29, -19),
            ("dead_tree_trunk", -17, 12), ("dead_tree_trunk", 23, -12),
            ("wooden_barrels_01", -12, -6)]):
        if not (MODELS / f"ph_{asset}/model.sdf").is_file():
            raise FileNotFoundError(f"Existing downloaded model required: ph_{asset}")
        include(f"sample_{i}", f"ph_{asset}", x, y, rng.uniform(-math.pi,math.pi))
    rover(world)
    vehicle = world.find("model[@name='survey_rover']")
    vehicle.find("pose").text = "0 0 0.45 0 0 0"
    for link in vehicle.findall("link"):
        for child in list(link):
            if child.tag == "sensor" or child.get("name", "").startswith("mast_"):
                link.remove(child)
    for visual in world.findall(".//visual"):
        node(visual, "cast_shadows", "false")
    write_xml(WORLD, sdf)
    MANIFEST.write_text(json.dumps(dict(seed=SEED, size_m=80, grid_m=STEP,
        terrain_vertices=len(vertices), terrain_triangles=sum(map(len,buckets))//3,
        surface_triangles=dict(zip(SURFACES, (len(b)//3 for b in buckets))),
        paths={"concrete": [[-39+i*STEP,0] for i in range(157)], "dirt": DIRT[:-2]},
        potholes=POTHOLES, rut_y_range=[15,34], spawn=[0,0,.45], placements=placements,
        limitations=["Procedural broadleaf crowns, not scanned trees", "Untextured diffuse road materials with UVs, not PBR",
                      "Tree trunk proxy collisions only; downloaded assets retain their original proxy collisions"]),
        indent=2) + "\n", encoding="utf-8")
    print(f"Built {WORLD.relative_to(ROOT)}: {len(vertices)} vertices, 51200 triangles, 44 trees, 5 reused assets")


if __name__ == "__main__":
    build()
