"""Coordinate-independent metre/Z-up meshes and native ScenarioObject definitions.

Only placement metadata contains Italy coordinates. No BeamNG dependency is needed
to build meshes; scenario_objects imports BeamNGpy only when explicitly called.
"""
from bisect import bisect_right
from collections import defaultdict
import json
import math
from pathlib import Path
import random
import xml.etree.ElementTree as ET

ART = "/art/shapes/nizhen_rural/"


def sub(a, b):
    return tuple(x-y for x, y in zip(a, b))


def unit(v):
    n = math.sqrt(sum(x*x for x in v))
    if n < 1e-12:
        raise ValueError("Degenerate vector")
    return tuple(x/n for x in v)


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


class Route:
    """Shape-preserving cubic station interpolation, with no height overshoot."""

    def __init__(self, nodes, config):
        if len(nodes) < 2:
            raise ValueError("A route needs two nodes")
        self.config = config
        self.origin = tuple(nodes[0][:3])
        self.nodes = [sub(n[:3], self.origin) for n in nodes]
        self.stations = [0.0]
        for a, b in zip(nodes, nodes[1:]):
            d = math.hypot(b[0]-a[0], b[1]-a[1])
            if d < 0.01:
                raise ValueError("Coincident road nodes")
            self.stations.append(self.stations[-1]+d)
        self.length = self.stations[-1]
        self.slopes = []
        h = [b-a for a, b in zip(self.stations, self.stations[1:])]
        for axis in range(3):
            d = [(b[axis]-a[axis])/step for a, b, step in zip(self.nodes, self.nodes[1:], h)]
            m = [d[0]]
            for i in range(1, len(d)):
                if d[i-1]*d[i] <= 0:
                    m.append(0.0)
                else:
                    w1, w2 = 2*h[i]+h[i-1], h[i]+2*h[i-1]
                    m.append((w1+w2)/(w1/d[i-1]+w2/d[i]))
            m.append(d[-1])
            self.slopes.append(m)

    def center(self, s):
        s = min(self.length, max(0.0, s))
        i = min(len(self.nodes)-2, bisect_right(self.stations, s)-1)
        h = self.stations[i+1]-self.stations[i]
        t = (s-self.stations[i])/h
        return tuple((2*t**3-3*t*t+1)*self.nodes[i][k]
                     +(t**3-2*t*t+t)*h*self.slopes[k][i]
                     +(-2*t**3+3*t*t)*self.nodes[i+1][k]
                     +(t**3-t*t)*h*self.slopes[k][i+1] for k in range(3))

    def tangent(self, s):
        a, b = self.center(s-0.001), self.center(s+0.001)
        return unit((b[0]-a[0], b[1]-a[1], 0))

    def lift(self, s):
        t = min(1.0, max(0.0, min(s, self.length-s)/self.config["transition_length_m"]))
        return self.config["overlay_height_m"]*t*t*(3-2*t)

    def point(self, s, lateral=0.0, height=0.0, overlay=True):
        p, t = self.center(s), self.tangent(s)
        return (p[0]-t[1]*lateral, p[1]+t[0]*lateral,
                p[2]+height+(self.lift(s) if overlay else 0.0))

    def world(self, p):
        return [a+b for a, b in zip(p, self.origin)]


class Mesh:
    def __init__(self, name):
        self.name = name
        self.vertices = []
        self.normals = []
        self.uvs = []
        self.groups = defaultdict(list)

    def triangle(self, a, b, c, material, uv=None):
        normal = unit(cross(sub(b, a), sub(c, a)))
        start = len(self.vertices)
        self.vertices.extend((a, b, c))
        self.normals.extend((normal, normal, normal))
        self.uvs.extend(uv or ((0, 0), (1, 0), (1, 1)))
        self.groups[material].extend((start, start+1, start+2))

    def quad(self, a, b, c, d, material, uv=None):
        uv = uv or ((0, 0), (1, 0), (1, 1), (0, 1))
        self.triangle(a, b, c, material, uv[:3])
        self.triangle(a, c, d, material, (uv[0], uv[2], uv[3]))

    def box(self, center, size, material):
        x, y, z = center
        a, b, c = (v/2 for v in size)
        p = [(x+dx*a, y+dy*b, z+dz*c) for dz in (-1, 1) for dy in (-1, 1) for dx in (-1, 1)]
        for ids in ((0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)):
            self.quad(*(p[i] for i in ids), material)

    def cylinder(self, center, rings, material, sides=32):
        """Lathed profile of (height, radius), including closed end caps."""
        x, y, z = center
        pts = [[(x+r*math.cos(2*math.pi*i/sides), y+r*math.sin(2*math.pi*i/sides), z+h)
                for i in range(sides)] for h, r in rings]
        for k in range(len(pts)-1):
            for i in range(sides):
                j = (i+1) % sides
                self.quad(pts[k][i], pts[k][j], pts[k+1][j], pts[k+1][i], material,
                          ((i/sides, rings[k][0]), ((i+1)/sides, rings[k][0]),
                           ((i+1)/sides, rings[k+1][0]), (i/sides, rings[k+1][0])))
        for i in range(sides):
            j = (i+1) % sides
            self.triangle((x, y, z+rings[0][0]), pts[0][j], pts[0][i], material)
            self.triangle((x, y, z+rings[-1][0]), pts[-1][i], pts[-1][j], material)

    def beam(self, a, b, width, material):
        direction = unit(sub(b, a))
        u = unit(cross(direction, (0, 0, 1)))
        v = unit(cross(direction, u))
        rings = [[tuple(p[k]+width/2*(su*u[k]+sv*v[k]) for k in range(3))
                  for su, sv in ((-1, -1), (1, -1), (1, 1), (-1, 1))] for p in (a, b)]
        self.quad(*reversed(rings[0]), material)
        self.quad(*rings[1], material)
        for i in range(4):
            j = (i+1) % 4
            self.quad(rings[0][i], rings[0][j], rings[1][j], rings[1][i], material)

    def bounds(self):
        return {"min": [min(v[i] for v in self.vertices) for i in range(3)],
                "max": [max(v[i] for v in self.vertices) for i in range(3)]}

    def write_dae(self, path):
        ns = "http://www.collada.org/2005/11/COLLADASchema"
        ET.register_namespace("", ns)
        def elem(parent, tag, text=None, **attrs):
            node = ET.SubElement(parent, "{"+ns+"}"+tag, {k: str(v) for k, v in attrs.items()})
            node.text = text
            return node
        root = ET.Element("{"+ns+"}COLLADA", version="1.4.1")
        asset = elem(root, "asset")
        elem(asset, "created", "2026-09-10T00:00:00Z")
        elem(asset, "modified", "2026-09-10T00:00:00Z")
        elem(asset, "unit", name="meter", meter="1")
        elem(asset, "up_axis", "Z_UP")
        effects = elem(root, "library_effects")
        materials = elem(root, "library_materials")
        for mat in self.groups:
            effect = elem(effects, "effect", id=mat+"-fx")
            tech = elem(elem(effect, "profile_COMMON"), "technique", sid="common")
            elem(elem(elem(tech, "lambert"), "diffuse"), "color", "0.5 0.5 0.5 1")
            elem(elem(materials, "material", id=mat, name=mat), "instance_effect", url="#"+mat+"-fx")
        geo = elem(elem(root, "library_geometries"), "geometry", id="mesh", name=self.name)
        mesh = elem(geo, "mesh")
        for name, values, params in (("positions", self.vertices, "XYZ"), ("normals", self.normals, "XYZ"), ("uv", self.uvs, "ST")):
            source = elem(mesh, "source", id=name)
            elem(source, "float_array", " ".join(f"{x:.7f}" for v in values for x in v), id=name+"-array", count=len(values)*len(params))
            accessor = elem(elem(source, "technique_common"), "accessor", source="#"+name+"-array", count=len(values), stride=len(params))
            for param in params:
                elem(accessor, "param", name=param, type="float")
        elem(elem(mesh, "vertices", id="vertices"), "input", semantic="POSITION", source="#positions")
        for mat, indices in self.groups.items():
            tri = elem(mesh, "triangles", material=mat, count=len(indices)//3)
            elem(tri, "input", semantic="VERTEX", source="#vertices", offset="0")
            elem(tri, "input", semantic="NORMAL", source="#normals", offset="1")
            elem(tri, "input", semantic="TEXCOORD", source="#uv", offset="2", set="0")
            elem(tri, "p", " ".join(str(i) for i in indices for _ in range(3)))
        scene = elem(elem(root, "library_visual_scenes"), "visual_scene", id="Scene", name="Scene")
        node = elem(scene, "node", id=self.name+"300", name=self.name+"300", type="NODE")
        instance = elem(node, "instance_geometry", url="#mesh")
        binding = elem(elem(instance, "bind_material"), "technique_common")
        for mat in self.groups:
            im = elem(binding, "instance_material", symbol=mat, target="#"+mat)
            elem(im, "bind_vertex_input", semantic="UVMap", input_semantic="TEXCOORD", input_set="0")
        elem(elem(root, "scene"), "instance_visual_scene", url="#Scene")
        ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def build_geometry(route):
    c = route.config
    half = c["lane_width_m"]/2
    start, end = c["cobble_start_m"], c["cobble_end_m"]
    if not (5 < start < end < route.length-5) or not (0 < c["cobble_relief_max_m"] <= 0.04):
        raise ValueError("Invalid course extents or stone relief")
    meshes = {}
    seams = {}
    for name, lo, hi, mat in (("concrete_entry", 0, start, "nz_concrete"),
                              ("cobble_bedding", start, end, "nz_mortar"),
                              ("concrete_exit", end, route.length, "nz_concrete")):
        m = Mesh(name)
        count = math.ceil((hi-lo)/c["station_step_m"])
        stations = [lo+(hi-lo)*i/count for i in range(count+1)]
        for a, b in zip(stations, stations[1:]):
            # Right edge to left edge winding faces upward. Shared boundaries
            # use exactly the same route.point function for every road section.
            m.quad(route.point(a, -half), route.point(b, -half), route.point(b, half), route.point(a, half), mat,
                   ((0, a/3), (0, b/3), (c["lane_width_m"]/3, b/3), (c["lane_width_m"]/3, a/3)))
            for side in (-half, half):
                p, q = route.point(a, side), route.point(b, side)
                r, t = route.point(b, side, -0.04), route.point(a, side, -0.04)
                m.quad(*((t, r, q, p) if side == -half else (p, q, r, t)), mat)
        seams[name] = [[route.point(s, -half), route.point(s, half)] for s in (lo, hi)]
        meshes[name] = m
    rng = random.Random(c["seed"])
    stones = Mesh("cobbles_merged")
    reliefs = []
    rows, cols = c["cobble_rows"], c["cobble_columns"]
    ds, dt = (end-start)/rows, c["lane_width_m"]/cols
    for row in range(rows):
        for col in range(cols):
            s0, s1 = start+row*ds+0.003, start+(row+1)*ds-0.003
            t0, t1 = -half+col*dt+0.003, -half+(col+1)*dt-0.003
            h = rng.uniform(0.012, c["cobble_relief_max_m"])
            h *= min(1, (s0-start)/2, (end-s1)/2)
            reliefs.append(h)
            bevel = min(0.025, dt/6)
            base = [route.point(s, t) for s, t in ((s0, t0), (s1, t0), (s1, t1), (s0, t1))]
            top = [route.point(s, t, h) for s, t in ((s0+bevel, t0+bevel), (s1-bevel, t0+bevel), (s1-bevel, t1-bevel), (s0+bevel, t1-bevel))]
            mat = "nz_stone_"+str(rng.randrange(4))
            stones.quad(*top, mat)
            for i in range(4):
                j = (i+1) % 4
                stones.quad(base[i], base[j], top[j], top[i], mat)
    meshes[stones.name] = stones
    fence = Mesh("timber_fences")
    gaps = c["fence_gaps_m"]
    for side in (-1, 1):
        lateral = side*c["fence_offset_m"]
        for s in range(18, int(route.length)-8, 3):
            if any(s < hi and s+3 > lo for lo, hi in gaps):
                continue
            p = route.point(s, lateral, overlay=False)
            fence.box((p[0], p[1], p[2]+0.52), (0.13, 0.13, 1.2), "nz_wood")
            for height in (0.4, 0.92):
                fence.beam(route.point(s, lateral, height, overlay=False),
                           route.point(s+3, lateral, height, overlay=False), 0.09, "nz_wood")
    meshes[fence.name] = fence
    # Small farm assets are authored around their own origins for reuse.
    tank = Mesh("water_tank")
    tank.cylinder((0, 0, 0), [(0, .92), (.12, .96), (.2, .9), (1.9, .9), (2.05, .82), (2.12, .4)], "nz_zinc", 48)
    for z in (.28, .62, .96, 1.3, 1.64, 1.9):
        tank.cylinder((0, 0, z), [(0, .905), (.025, .93), (.05, .905)], "nz_zinc", 48)
    tank.cylinder((0, 0, 2.12), [(0, .22), (.08, .22)], "nz_iron")
    tank.beam((.84, 0, .22), (1.12, 0, .22), .07, "nz_iron")
    meshes[tank.name] = tank
    barrel = Mesh("barrel")
    barrel.cylinder((0, 0, 0), [(0, .28), (.06, .30), (.43, .33), (.82, .30), (.88, .28)], "nz_barrel")
    for z in (.12, .65):
        barrel.cylinder((0, 0, z), [(0, .314), (.055, .32)], "nz_iron")
    barrel.cylinder((.12, 0, .88), [(0, .04), (.018, .04)], "nz_iron", 16)
    meshes[barrel.name] = barrel
    trough = Mesh("stone_trough")
    trough.box((0, 0, .12), (1.6, .8, .24), "nz_mortar")
    for y in (-.35, .35):
        trough.box((0, y, .38), (1.6, .1, .52), "nz_stone_1")
    for x in (-.75, .75):
        trough.box((x, 0, .38), (.1, .6, .52), "nz_stone_1")
    meshes[trough.name] = trough
    return meshes, {"road_seams_local": seams, "stone_count": len(reliefs),
                    "stone_relief_min_m": min(reliefs), "stone_relief_max_m": max(reliefs)}


def object_definitions(route, meshes):
    """Return dicts accepted directly by beamngpy.ScenarioObject(**definition)."""
    defs = []
    for name in ("concrete_entry", "cobble_bedding", "concrete_exit", "cobbles_merged", "timber_fences"):
        defs.append(dict(oid="nz_"+name, name="nz_"+name, otype="TSStatic", pos=list(route.origin),
                         rot_quat=[0, 0, 0, 1], scale=[1, 1, 1], shapeName=ART+name+".dae",
                         collisionType="Visible Mesh", groundType="ASPHALT" if name != "timber_fences" else "WOOD"))
    for name, mesh, s, lateral in (("tank", "water_tank", 40, 2.85),
                                   ("barrel_a", "barrel", 38, 2.5), ("barrel_b", "barrel", 42, 2.5),
                                   ("barrel_c", "barrel", 92, -2.6), ("barrel_d", "barrel", 94, -2.7),
                                   ("trough", "stone_trough", 129, 2.6)):
        defs.append(dict(oid="nz_"+name, name="nz_"+name, otype="TSStatic", pos=route.world(route.point(s, lateral, overlay=False)),
                         rot_quat=[0, 0, 0, 1], scale=[1, 1, 1], shapeName=ART+mesh+".dae", collisionType="Visible Mesh"))
    return defs


def scenario_objects(manifest_path=None):
    """Construct, but do not add/spawn/install, the generated native objects."""
    from beamngpy import ScenarioObject
    path = Path(manifest_path) if manifest_path else Path(__file__).parent/"generated"/"map_objects.json"
    data = json.loads(path.read_text(encoding="utf-8"))
    return [ScenarioObject(**definition) for definition in data["objects"]]
