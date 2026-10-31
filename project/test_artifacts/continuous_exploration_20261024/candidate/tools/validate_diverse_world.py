"""Offline checks of the generated files, including actual triangulated road grades."""
import argparse
from collections import Counter, deque
import copy
import hashlib
import json
import math
import xml.etree.ElementTree as ET

from build_diverse_world import WORLD, MODELS, MANIFEST, N, STEP, build

NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def mesh_check(root, terrain=False):
    sources = {}
    for source in root.findall(".//c:mesh/c:source", NS):
        array = source.find("c:float_array", NS)
        values = list(map(float, array.text.split()))
        accessor = source.find("c:technique_common/c:accessor", NS)
        stride = int(accessor.get("stride"))
        require(len(values) == int(array.get("count")) == int(accessor.get("count"))*stride,
                "Source count mismatch")
        require(all(math.isfinite(v) for v in values), "Nonfinite mesh data")
        sources[source.get("id")] = [values[i:i+stride] for i in range(0,len(values),stride)]
    vertices = sources["positions"]
    normals = sources["normals"]
    require(all(abs(math.sqrt(sum(v*v for v in n))-1) < 2e-5 for n in normals), "Normals must be unit length")
    ids = {e.get("id") for e in root.iter() if e.get("id")}
    for element in root.iter():
        for key in ("source", "url", "target"):
            value = element.get(key, "")
            if value.startswith("#"):
                require(value[1:] in ids, f"Broken DAE reference {value}")
    bindings = {m.get("symbol") for m in root.findall(".//c:instance_material", NS)}
    faces, road_faces, edges = [], [], Counter()
    for tri in root.findall(".//c:triangles", NS):
        inputs = {e.get("semantic"): e for e in tri.findall("c:input", NS)}
        require({"VERTEX", "NORMAL"} <= inputs.keys(), "Missing normal input")
        require(tri.get("material") in bindings, "Unbound material")
        if terrain:
            require("TEXCOORD" in inputs, "Terrain missing UVs")
        stride = max(int(e.get("offset")) for e in inputs.values())+1
        indices = list(map(int, tri.findtext("c:p", namespaces=NS).split()))
        require(len(indices) == int(tri.get("count"))*3*stride, "Triangle count mismatch")
        for semantic, inp in inputs.items():
            source = "positions" if semantic == "VERTEX" else inp.get("source")[1:]
            require(all(0 <= i < len(sources[source]) for i in indices[int(inp.get('offset'))::stride]),
                    "Index out of bounds")
        for t in range(0,len(indices),3*stride):
            face = tuple(indices[t+k*stride+int(inputs['VERTEX'].get('offset'))] for k in range(3))
            a,b,c = (vertices[i] for i in face)
            u,v = [b[k]-a[k] for k in range(3)], [c[k]-a[k] for k in range(3)]
            cross = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
            require(sum(v*v for v in cross) > 1e-14, "Degenerate triangle")
            for k in range(3):
                normal = normals[indices[t+k*stride+int(inputs['NORMAL'].get('offset'))]]
                require(sum(cross[j]*normal[j] for j in range(3)) > 0, "Normal opposes triangle")
            if terrain:
                require(cross[2] > 0, "Terrain winding not upward")
                for k in range(3):
                    edges[tuple(sorted((face[k],face[(k+1)%3])))] += 1
            faces.append(face)
            if tri.get("material") in ("mat0", "mat1", "mat5"):
                road_faces.append(face)
    require(len(faces) > 0, "Empty mesh")
    if terrain:
        require(len(vertices) == N*N and len(faces) == 2*(N-1)**2, "Terrain resolution mismatch")
        require(len(set(tuple(sorted(f)) for f in faces)) == len(faces), "Duplicate terrain faces")
        require(set(edges.values()) == {1,2} and sum(v == 1 for v in edges.values()) == 4*(N-1),
                "Terrain holes or nonmanifold edges")
        for j in range(N):
            for i in range(N):
                require(vertices[j*N+i][:2] == [i*STEP-40,j*STEP-40], "Terrain grid mismatch")
        # Require edge-connected road triangles, including the concrete/dirt junction.
        owners, graph = {}, [set() for _ in road_faces]
        for index, face in enumerate(road_faces):
            for k in range(3):
                edge = tuple(sorted((face[k],face[(k+1)%3])))
                if edge in owners:
                    other = owners[edge]
                    graph[index].add(other)
                    graph[other].add(index)
                owners[edge] = index
        seen, queue = {0}, deque([0])
        while queue:
            for nxt in graph[queue.popleft()]:
                if nxt not in seen:
                    seen.add(nxt)
                    queue.append(nxt)
        require(len(seen) == len(road_faces), "Disconnected road surface")
    return vertices, len(faces)


def references(path, seen):
    path = path.resolve()
    if path in seen:
        return
    require(path.is_file(), f"Missing reference: {path}")
    seen.add(path)
    if path.suffix not in (".sdf", ".dae", ".config"):
        return
    root = ET.parse(path).getroot()
    for element in root.iter():
        tag = element.tag.split('}')[-1]
        if tag == "uri" or (tag == "init_from" and element.text and not element.text.startswith('#')):
            uri = element.text.strip()
            if tag == "init_from" and root.find(f".//*[@id='{uri}']") is not None:
                continue  # COLLADA surface -> image ID, not a file.
            target = MODELS / uri[8:] if uri.startswith("model://") else path.parent / uri
            if target.is_dir():
                references(target / "model.config", seen)
                target /= "model.sdf"
            references(target, seen)
        elif tag == "sdf" and element.text and element.text.strip().endswith('.sdf'):
            references(path.parent / element.text.strip(), seen)


def ground(vertices, x, y):
    fx, fy = (x+40)/STEP, (y+40)/STEP
    i,j = min(N-2,int(fx)), min(N-2,int(fy))
    u,v = fx-i,fy-j
    a,b,c,d = (vertices[k][2] for k in (j*N+i,j*N+i+1,(j+1)*N+i+1,(j+1)*N+i))
    return a+(b-a)*u+(c-b)*v if v <= u else a+(c-d)*u+(d-a)*v


def validate():
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    terrain = ET.parse(MODELS / "diverse_terrain/visual.dae").getroot()
    vertices, triangles = mesh_check(terrain, True)
    total = triangles
    for i in range(8):
        total += mesh_check(ET.parse(MODELS / f"diverse_tree_{i}/visual.dae").getroot())[1]
    seen = set()
    references(WORLD, seen)
    world = ET.parse(WORLD).getroot().find("world")
    require(world.findtext("scene/shadows") == "false", "Scene shadows enabled")
    require(all(e.text == 'false' for e in world.findall('.//cast_shadows')), "Shadows enabled")
    require(not world.findall('.//sensor'), "Unexpected sensor")
    vehicle = world.find("model[@name='survey_rover']")
    require(vehicle is not None and vehicle.findtext('pose') == '0 0 0.45 0 0 0', "Unsafe spawn pose")
    require(len(vehicle.findall('joint')) == 4, "Rover joints missing")
    require(all(abs(ground(vertices,x*.5,y*.5)) < 1e-6 for x in range(-10,11) for y in range(-3,4)),
            "Spawn road not flat")
    grades = {}
    for name, path in manifest['paths'].items():
        samples = []
        for a,b in zip(path,path[1:]):
            for k in range(4):
                samples.append([a[j]+(b[j]-a[j])*k/4 for j in range(2)])
        samples.append(path[-1])
        slopes = [(ground(vertices,*b)-ground(vertices,*a))/math.dist(a,b) for a,b in zip(samples,samples[1:])]
        maximum = max(map(abs,slopes))
        jump = max(abs(b-a) for a,b in zip(slopes,slopes[1:]))
        require(maximum < .30, f"Excessive {name} path grade: {maximum}")
        require(jump < .18, f"Abrupt {name} grade transition: {jump}")
        grades[name] = dict(max_grade=round(maximum,6), max_grade_step=round(jump,6))
    for x,y,rx,ry,depth in manifest['potholes']:
        require(math.hypot(x,y) > 15, "Pothole near spawn")
        require(ground(vertices,x,y) < -.07, "Pothole lacks geometric depth")
    for y in (22,26,30):
        x = 12*math.sin(y/16)
        center = ground(vertices,x,y)
        mean_tracks = (ground(vertices,x-.65,y)+ground(vertices,x+.65,y))/2
        require(center-mean_tracks > .025, "Ruts lack geometric depth")
    require(len(manifest['placements']) == 49, "Placement count mismatch")
    for placement in manifest['placements']:
        x,y,z = placement['position']
        require(math.hypot(x,y) > 8, "Obstacle near spawn")
        require(abs(ground(vertices,x,y)-z) < .05, "Object ground placement mismatch")
        require(world.find(f"include[name='{placement['name']}']") is not None, "Missing placed model")
    report = dict(terrain_vertices=len(vertices), terrain_triangles=triangles,
                  all_new_mesh_triangles=total, resolved_files=len(seen), paths=grades,
                  elevation_range=[round(min(v[2] for v in vertices),4),round(max(v[2] for v in vertices),4)],
                  trees=44, reused_assets=5, sensors=0, shadows=False)
    print(json.dumps(report, indent=2))
    return terrain


def self_test(terrain):
    for defect in ('normal', 'triangle', 'reference'):
        broken = copy.deepcopy(terrain)
        if defect == 'normal':
            array = broken.find(".//c:source[@id='normals']/c:float_array", NS)
            values = array.text.split()
            values[:3] = ['0','0','0']
            array.text = ' '.join(values)
        elif defect == 'triangle':
            array = broken.find('.//c:triangles/c:p', NS)
            values = array.text.split()
            values[3:6] = values[:3]
            array.text = ' '.join(values)
        else:
            broken.find('.//c:instance_geometry', NS).set('url','#missing')
        try:
            mesh_check(broken, True)
        except ValueError:
            pass
        else:
            raise ValueError(f'Validator accepted {defect} corruption')
    print('PASS: rejected zero normal, degenerate triangle, broken DAE reference')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('--reproducible', action='store_true', help='Regenerate only diverse outputs and compare SHA-256')
    args = parser.parse_args()
    terrain = validate()
    if args.self_test:
        self_test(terrain)
    if args.reproducible:
        paths = [WORLD, MANIFEST] + sorted(p for d in MODELS.glob('diverse_*') for p in d.rglob('*') if p.is_file())
        before = {p: hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
        build()
        require(all(hashlib.sha256(p.read_bytes()).hexdigest() == digest for p,digest in before.items()),
                'Generation is not deterministic')
        print(f'PASS: reproducible SHA-256 for {len(paths)} generated files')
