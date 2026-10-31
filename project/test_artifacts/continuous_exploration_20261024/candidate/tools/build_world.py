"""Generate original, deterministic Gazebo assets using only Python's stdlib."""
import json
import math
import random
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "assets"
SEED = 19308
ROADS = [((-140, -90), (115, -90)), ((115, -90), (115, 95)),
         ((115, 95), (-110, 95)), ((-110, 95), (-110, -90)),
         ((-110, 0), (115, 0)), ((15, -90), (15, 95))]
OBJECTS = []


def segment_distance(x, y, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    t = max(0, min(1, ((x-a[0])*dx + (y-a[1])*dy)/(dx*dx+dy*dy)))
    return math.hypot(x-a[0]-t*dx, y-a[1]-t*dy)


def road_distance(x, y):
    return min(segment_distance(x, y, a, b) for a, b in ROADS)


def height(x, y):
    hill = 3.8 * math.exp(-((x-63)**2+(y-52)**2)/2400)
    hill += 1.8 * math.exp(-((x+48)**2+(y-55)**2)/1900)
    # Roughness fades out at road edges; the northern road remains undulating.
    blend = max(0, min(1, (road_distance(x, y)-2.8)/4))
    rough_zone = max(0, min(1, (y-8)/18))
    rough = rough_zone * blend * (0.13*math.sin(x*0.85)*math.sin(y*0.7)
                                  + 0.08*math.sin(x*1.8+y*0.9))
    return hill + rough


def node(parent, tag, text=None, **attrs):
    n = ET.SubElement(parent, tag, {k: str(v) for k, v in attrs.items()})
    if text is not None:
        n.text = str(text)
    return n


def write_xml(path, root):
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def terrain():
    """One surface for road, dirt and grass; no stacked coplanar road meshes."""
    folder = OUT / "models" / "wildos_terrain"
    folder.mkdir(parents=True, exist_ok=True)
    vertices = []
    for j in range(301):
        for i in range(301):
            x, y = i-150, j-150
            vertices.extend((x, y, height(x, y)))
    buckets = [[] for _ in range(5)]
    for j in range(300):
        for i in range(300):
            x, y = i-149.5, j-149.5
            d = road_distance(x, y)
            if d < 3:
                material = 0 if y < 12 else 1
            elif y > 15 and x > -65:
                material = 1 if (i+j) % 7 else 2
            else:
                material = 3 if (i*17+j*23) % 11 else 4
            a = j*301+i
            buckets[material].extend((a, a+1, a+302, a, a+302, a+301))
    # Area-weighted vertex normals, including faces across material boundaries.
    normals = [0.0] * len(vertices)
    for indices in buckets:
        for t in range(0, len(indices), 3):
            a, b, c = (3*index for index in indices[t:t+3])
            u = [vertices[b+k]-vertices[a+k] for k in range(3)]
            v = [vertices[c+k]-vertices[a+k] for k in range(3)]
            n = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
            for index in (a, b, c):
                for k in range(3):
                    normals[index+k] += n[k]
    for index in range(0, len(normals), 3):
        length = math.sqrt(sum(v*v for v in normals[index:index+3]))
        assert length > 0
        normals[index:index+3] = [v/length for v in normals[index:index+3]]
    dae = ET.Element("COLLADA", xmlns="http://www.collada.org/2005/11/COLLADASchema", version="1.4.1")
    asset = node(dae, "asset")
    node(asset, "created", "2026-09-08T00:00:00Z")
    node(asset, "modified", "2026-09-08T00:00:00Z")
    node(asset, "unit", name="meter", meter="1")
    node(asset, "up_axis", "Z_UP")
    colors = [(0.49,0.48,0.44), (0.39,0.29,0.17), (0.45,0.35,0.23),
              (0.23,0.32,0.12), (0.28,0.35,0.15)]
    effects, materials = node(dae, "library_effects"), node(dae, "library_materials")
    for k, color in enumerate(colors):
        effect = node(effects, "effect", id=f"e{k}")
        technique = node(node(effect, "profile_COMMON"), "technique", sid="common")
        phong = node(technique, "phong")
        node(node(phong, "diffuse"), "color", " ".join(map(str, (*color,1))))
        mat = node(materials, "material", id=f"m{k}", name=f"surface{k}")
        node(mat, "instance_effect", url=f"#e{k}")
    mesh = node(node(node(dae, "library_geometries"), "geometry", id="ground"), "mesh")
    source = node(mesh, "source", id="positions")
    node(source, "float_array", " ".join(f"{v:.5f}" for v in vertices), id="positions-array", count=len(vertices))
    accessor = node(node(source, "technique_common"), "accessor", source="#positions-array", count=len(vertices)//3, stride="3")
    for axis in "XYZ":
        node(accessor, "param", name=axis, type="float")
    node(node(mesh, "vertices", id="verts"), "input", semantic="POSITION", source="#positions")
    source = node(mesh, "source", id="normals")
    node(source, "float_array", " ".join(f"{v:.7f}" for v in normals), id="normals-array", count=len(normals))
    accessor = node(node(source, "technique_common"), "accessor", source="#normals-array", count=len(normals)//3, stride="3")
    for axis in "XYZ":
        node(accessor, "param", name=axis, type="float")
    for k, indices in enumerate(buckets):
        tri = node(mesh, "triangles", material=f"mat{k}", count=len(indices)//3)
        node(tri, "input", semantic="VERTEX", source="#verts", offset="0")
        node(tri, "input", semantic="NORMAL", source="#normals", offset="1")
        node(tri, "p", " ".join(f"{i} {i}" for i in indices))
    scene = node(node(dae, "library_visual_scenes"), "visual_scene", id="Scene")
    instance = node(node(scene, "node", id="terrain-node"), "instance_geometry", url="#ground")
    common = node(node(instance, "bind_material"), "technique_common")
    for k in range(len(colors)):
        node(common, "instance_material", symbol=f"mat{k}", target=f"#m{k}")
    node(node(dae, "scene"), "instance_visual_scene", url="#Scene")
    write_xml(folder / "terrain.dae", dae)
    sdf = ET.Element("sdf", version="1.8")
    model = node(sdf, "model", name="wildos_terrain")
    node(model, "static", "true")
    link = node(model, "link", name="ground")
    for kind in ("visual", "collision"):
        mesh_node = node(node(node(link, kind, name=kind), "geometry"), "mesh")
        node(mesh_node, "uri", "model://wildos_terrain/terrain.dae")
    write_xml(folder / "model.sdf", sdf)
    config = ET.Element("model")
    node(config, "name", "WildOS Original Terrain")
    node(config, "version", "1.0")
    node(config, "sdf", "model.sdf", version="1.8")
    node(config, "description", "Original 300m terrain, shared visual and collision surface.")
    write_xml(folder / "model.config", config)


def primitive(link, name, shape, dimensions, pose, color, collision=True):
    for kind in (["visual", "collision"] if collision else ["visual"]):
        part = node(link, kind, name=f"{name}_{kind}")
        node(part, "pose", " ".join(map(str, pose)))
        geom = node(node(part, "geometry"), shape)
        if shape == "box":
            node(geom, "size", " ".join(map(str, dimensions)))
        elif shape == "cylinder":
            node(geom, "radius", dimensions[0])
            node(geom, "length", dimensions[1])
        elif shape == "sphere":
            node(geom, "radius", dimensions[0])
        if kind == "visual":
            mat = node(part, "material")
            for channel in ("ambient", "diffuse"):
                node(mat, channel, " ".join(map(str, (*color, 1))))


def model_at(world, name, x, y, yaw=0, category="obstacle"):
    model = node(world, "model", name=name)
    node(model, "static", "true")
    z = height(x, y)
    node(model, "pose", f"{x} {y} {z:.5f} 0 0 {yaw}")
    OBJECTS.append(dict(id=name, category=category, position=[x,y,z], yaw=yaw))
    return node(model, "link", name="body")


def box(world, name, x, y, size, color, yaw=0, category="obstacle"):
    link = model_at(world, name, x, y, yaw, category)
    primitive(link, "body", "box", size, (0,0,size[2]/2,0,0,0), color)
    return link


def fence(world, prefix, a, b):
    length = math.dist(a, b)
    yaw = math.atan2(b[1]-a[1], b[0]-a[0])
    pieces = math.ceil(length/2)
    for i in range(pieces):
        t = (i+0.5)/pieces
        x, y = a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1])
        link = model_at(world, f"{prefix}_{i}", x, y, yaw, "fence")
        primitive(link, "post", "box", (.12,.12,1.5), (0,0,.75,0,0,0), (.3,.24,.17))
        for k in range(3):
            primitive(link, f"rail{k}", "box", (length/pieces,.09,.1), (0,0,.35+k*.48,0,0,0), (.48,.39,.26))


def rover(world):
    """Generic prototype, explicitly not a calibrated Husky model."""
    model = node(world, "model", name="survey_rover")
    node(model, "pose", f"-125 -90 {height(-125,-90)+.45} 0 0 0")
    base = node(model, "link", name="base_link")
    inertial = node(base, "inertial")
    node(inertial, "mass", "35")
    inertia = node(inertial, "inertia")
    for key, value in dict(ixx=1.6,iyy=3.0,izz=3.6,ixy=0,ixz=0,iyz=0).items():
        node(inertia, key, value)
    primitive(base, "chassis", "box", (1,.65,.24), (0,0,0,0,0,0), (.85,.56,.08))
    primitive(base, "mast", "cylinder", (.035,.7), (0,0,.47,0,0,0), (.18,.19,.20))
    for front, x in (("front",.34),("rear",-.34)):
        for side, y in (("left",.40),("right",-.40)):
            name = f"{front}_{side}"
            link = node(model, "link", name=name)
            node(link, "pose", f"{x} {y} -.17 0 0 0")
            inertial = node(link, "inertial")
            node(inertial, "mass", 2)
            inertia = node(inertial, "inertia")
            for key, value in dict(ixx=.04,iyy=.06,izz=.04,ixy=0,ixz=0,iyz=0).items():
                node(inertia, key, value)
            primitive(link, "tire", "cylinder", (.23,.15), (0,0,0,math.pi/2,0,0), (.065,.07,.075))
            joint = node(model, "joint", name=f"{name}_joint", type="revolute")
            node(joint, "parent", "base_link")
            node(joint, "child", name)
            node(node(joint, "axis"), "xyz", "0 1 0")
    drive = node(model, "plugin", filename="ignition-gazebo-diff-drive-system", name="ignition::gazebo::systems::DiffDrive")
    for front in ("front","rear"):
        node(drive, "left_joint", f"{front}_left_joint")
        node(drive, "right_joint", f"{front}_right_joint")
    for key, val in dict(wheel_separation=.8,wheel_radius=.23,topic="/rover/cmd_vel",odom_topic="/rover/wheel_odom",tf_topic="/rover/wheel_tf",frame_id="odom",child_frame_id="base_link",odom_publish_frequency=30,max_linear_velocity=1.5,max_angular_velocity=1.0).items():
        node(drive, key, val)
    for name, yaw in (("front",0),("left",math.pi/2),("right",-math.pi/2)):
        sensor = node(base, "sensor", name=f"camera_{name}", type="camera")
        node(sensor, "pose", f".25 0 .62 0 0 {yaw}")
        node(sensor, "topic", f"/rover/camera/{name}")
        node(sensor, "update_rate", 10)
        cam = node(sensor, "camera")
        node(cam, "horizontal_fov", 1.35)
        image = node(cam, "image")
        node(image, "width", 960)
        node(image, "height", 540)
        node(image, "format", "R8G8B8")
        clip = node(cam, "clip")
        node(clip, "near", .1)
        node(clip, "far", 350)
    sensor = node(base, "sensor", name="lidar", type="gpu_lidar")
    node(sensor, "pose", "0 0 .92 0 0 0")
    node(sensor, "topic", "/rover/lidar")
    node(sensor, "update_rate", 10)
    lidar = node(sensor, "lidar")
    scan = node(lidar, "scan")
    for axis, samples, lo, hi in (("horizontal",1024,-math.pi,math.pi),("vertical",64,-.55,.55)):
        part = node(scan, axis)
        for key, val in dict(samples=samples,resolution=1,min_angle=lo,max_angle=hi).items():
            node(part, key, val)
    rng = node(lidar, "range")
    for key, val in dict(min=.3,max=100,resolution=.02).items():
        node(rng, key, val)
    noise = node(lidar, "noise")
    node(noise, "type", "gaussian")
    node(noise, "mean", 0)
    node(noise, "stddev", .02)
    sensor = node(base, "sensor", name="imu", type="imu")
    node(sensor, "topic", "/rover/imu")
    node(sensor, "update_rate", 100)
    node(sensor, "imu")


def build():
    OBJECTS.clear()
    terrain()
    sdf = ET.Element("sdf", version="1.8")
    world = node(sdf, "world", name="wildos_outdoor")
    physics = node(world, "physics", name="physics", type="ignored")
    node(physics, "max_step_size", .002)
    node(physics, "real_time_factor", 1)
    node(world, "gravity", "0 0 -9.81")
    for short, classname in (("physics","Physics"),("user-commands","UserCommands"),("scene-broadcaster","SceneBroadcaster"),("sensors","Sensors"),("imu","Imu")):
        plugin = node(world, "plugin", filename=f"ignition-gazebo-{short}-system", name=f"ignition::gazebo::systems::{classname}")
        if short == "sensors":
            node(plugin, "render_engine", "ogre2")
    scene = node(world, "scene")
    node(scene, "ambient", "0.45 0.45 0.45 1")
    node(scene, "background", "0.62 0.77 0.91 1")
    node(scene, "shadows", "true")
    light = node(world, "light", name="sun", type="directional")
    node(light, "pose", "0 0 100 0 0 0")
    node(light, "direction", "-.4 -.3 -1")
    node(light, "diffuse", ".95 .92 .85 1")
    node(light, "specular", ".2 .2 .2 1")
    node(light, "cast_shadows", "true")
    node(node(world, "include"), "uri", "model://wildos_terrain")
    # Two separated enclosures leave a six metre visual gap across the road.
    for name, a, b in [("west_fence",(-60,-55),(-3,-55)),("east_fence",(3,-55),(70,-55)),
                       ("west_return",(-60,-55),(-60,-20)),("east_return",(70,-55),(70,-20)),
                       ("deadend_left",(33,12),(33,43)),("deadend_right",(43,12),(43,43)),
                       ("deadend_back",(33,43),(43,43))]:
        fence(world, name, a, b)
    for i, (a,b) in enumerate([((-149,-149),(149,-149)),((149,-149),(149,149)),((149,149),(-149,149)),((-149,149),(-149,-149))]):
        fence(world, f"boundary{i}", a,b)
    for name,x,y,color in [("house",-65,-115,(.76,.66,.48)),("barn",-90,64,(.55,.18,.12))]:
        link = box(world,name,x,y,(13,10,6),color,category="target")
        for side in (-1,1):
            primitive(link,f"roof{side}","box",(14,6,.25),(0,side*2.5,6.9,side*-.35,0,0),(.22,.23,.24))
        primitive(link,"door","box",(2,.15,3),(0,-5.08,1.5,0,0,0),(.12,.09,.06))
        for xw in (-4,4):
            primitive(link,f"window{xw}","box",(1.6,.16,1.4),(xw,-5.09,3.4,0,0,0),(.12,.28,.34))
    link = model_at(world,"water_tank",85,72,category="target")
    primitive(link,"tank","cylinder",(2.5,5),(0,0,3,0,0,0),(.57,.64,.69))
    primitive(link,"foundation","box",(5.5,5.5,1),(0,0,.5,0,0,0),(.4,.4,.38))
    link = box(world,"dumpster",-80,25,(2.8,1.7,1.6),(.08,.32,.18),category="target")
    primitive(link,"lid","box",(2.9,1.8,.15),(0,0,1.65,0,0,0),(.065,.075,.06))
    link = model_at(world,"orange_flag",55,112,category="target")
    primitive(link,"pole","cylinder",(.04,3),(0,0,1.5,0,0,0),(.6,.6,.6))
    primitive(link,"flag","box",(1.0,.045,.6),(.5,0,2.6,0,0,0),(1,.24,.015))
    for i,(x,y) in enumerate([(38,39),(76,-18),(-30,21)]):
        link = box(world,f"parked_vehicle{i}",x,y,(4.4,1.9,1.1),(.16,.25,.43))
        primitive(link,"cabin","box",(2.2,1.7,.85),(-.2,0,1.5,0,0,0),(.13,.22,.28))
        for xx in (-1.4,1.4):
            for yy in (-.94,.94):
                primitive(link,f"wheel{xx}{yy}","cylinder",(.4,.22),(xx,yy,.35,math.pi/2,0,0),(.045,.045,.045))
    rng = random.Random(SEED)
    for i in range(180):
        x,y = rng.uniform(-140,140),rng.uniform(-135,135)
        if road_distance(x,y)<5 or any(math.hypot(x-o['position'][0],y-o['position'][1])<9 for o in OBJECTS if o['category']=='target'):
            continue
        if y>10 and x>-50 and i%3:
            size = rng.uniform(.18,.55)
            link = model_at(world,f"rock{i}",x,y,category="rock")
            primitive(link,"stone","sphere",(size,),(0,0,size*.6,0,0,0),(.36,.34,.3))
        else:
            link = model_at(world,f"tree{i}",x,y,category="tree")
            primitive(link,"trunk","cylinder",(.22,3),(0,0,1.5,0,0,0),(.26,.17,.09))
            primitive(link,"crown","sphere",(1.7,),(0,0,3.5,0,0,0),(.14,.29,.09))
    rover(world)
    write_xml(OUT / "worlds" / "wildos_outdoor.sdf", sdf)
    # Separate physics-only world for software-rendered virtual machines.
    # Remove sensor systems and sensors, retaining vehicle control and odometry.
    for plugin in list(world.findall("plugin")):
        if plugin.get("name") in ("ignition::gazebo::systems::Sensors", "ignition::gazebo::systems::Imu"):
            world.remove(plugin)
    for link in world.findall(".//link"):
        for sensor in list(link.findall("sensor")):
            link.remove(sensor)
    write_xml(OUT / "worlds" / "wildos_physics_only.sdf", sdf)
    tasks = [dict(id="house_search",query="house",target="house",start=[-125,-90,0]),
             dict(id="tank_search",query="water tank",target="water_tank",start=[-100,-90,0]),
             dict(id="flag_search",query="orange flag",target="orange_flag",start=[15,-80,1.57]),
             dict(id="dumpster_search",query="dumpster",target="dumpster",start=[110,0,3.14])]
    manifest = dict(seed=SEED,extent_m=[300,300],terrain_grid_m=1,objects=OBJECTS,tasks=tasks,
                    warning="Evaluator-only truth. Do not supply target positions to navigation.")
    (OUT / "evaluation_truth.json").write_text(json.dumps(manifest,indent=2)+"\n",encoding="utf-8")
    preview(manifest)
    print(f"Built {len(OBJECTS)} static instances, terrain and sensor rover in {OUT}")


def preview(manifest):
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="1080" viewBox="0 0 1000 1080">',
           '<rect width="1000" height="1080" fill="#15201c"/>',
           '<text x="50" y="42" fill="#e4eedf" font-family="sans-serif" font-size="25">WILDOS FIELD / 300 x 300 m</text>',
           '<g transform="translate(50 960) scale(3 -3)">',
           '<rect x="0" y="0" width="300" height="300" fill="#647347"/>',
           '<rect x="85" y="165" width="215" height="135" fill="#88704f"/>']
    for a,b in ROADS:
        svg.append(f'<path d="M{a[0]+150},{a[1]+150} L{b[0]+150},{b[1]+150}" stroke="#c0b9a7" stroke-width="6"/>')
    colors = dict(fence="#dfc699",target="#ffb955",tree="#234a27",rock="#40433e",obstacle="#394d63")
    for o in manifest['objects']:
        x,y,_ = o['position']
        radius = 2.5 if o['category']=="target" else .8
        svg.append(f'<circle cx="{x+150}" cy="{y+150}" r="{radius}" fill="{colors[o["category"]]}"/>')
    svg.append('</g>')
    for o in manifest['objects']:
        if o['category']=="target":
            x,y,_ = o['position']
            svg.append(f'<text x="{50+3*(x+150)+10}" y="{960-3*(y+150)-10}" fill="#fff3d2" font-family="sans-serif" font-size="14">{o["id"]}</text>')
    svg.append('<text x="50" y="1000" fill="#c4d0c3" font-family="sans-serif" font-size="16">Layout preview only / not a Gazebo render. North is up. Seed 19308.</text>')
    svg.append('<text x="50" y="1030" fill="#c4d0c3" font-family="sans-serif" font-size="16">South: concrete roads | North: rolling dirt | Fences: gaps and dead ends</text></svg>')
    (OUT / "layout.svg").write_text("\n".join(svg),encoding="utf-8")


if __name__ == "__main__":
    build()
