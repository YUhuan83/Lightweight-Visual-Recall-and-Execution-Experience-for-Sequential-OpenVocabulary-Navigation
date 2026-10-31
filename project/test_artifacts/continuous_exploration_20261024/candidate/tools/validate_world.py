"""Offline structural checks; not a replacement for libsdformat or physics tests."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def validate():
    world = ET.parse(ROOT / "assets/worlds/wildos_outdoor.sdf").getroot().find("world")
    models = world.findall("model")
    names = [m.get("name") for m in models]
    assert len(names) == len(set(names)), "Duplicate model names"
    for pose in world.iter("pose"):
        numbers = list(map(float, pose.text.split()))
        assert len(numbers) == 6 and all(map(math.isfinite, numbers))
    for size in world.iter("size"):
        assert all(float(v) > 0 for v in size.text.split())
    for model in models:
        links = {link.get("name") for link in model.findall("link")}
        for joint in model.findall("joint"):
            assert joint.findtext("parent") in links
            assert joint.findtext("child") in links
    truth = json.loads((ROOT / "assets/evaluation_truth.json").read_text())
    assert len(truth["objects"]) == len(models)-1
    for task in truth["tasks"]:
        assert task["target"] in names
    terrain = ET.parse(ROOT / "assets/models/wildos_terrain/terrain.dae")
    ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
    array = terrain.find(".//c:float_array[@id='positions-array']", ns)
    coords = list(map(float, array.text.split()))
    assert len(coords) == int(array.get("count")) == 301*301*3
    assert all(map(math.isfinite, coords))
    normal_array = terrain.find(".//c:float_array[@id='normals-array']", ns)
    assert normal_array is not None, "DART collision meshes require explicit normals"
    normals = list(map(float, normal_array.text.split()))
    assert len(normals) == int(normal_array.get("count")) == len(coords)
    assert all(map(math.isfinite, normals))
    for i in range(0, len(normals), 3):
        assert abs(sum(v*v for v in normals[i:i+3])-1) < 1e-5
        assert normals[i+2] > 0
    triangles = 0
    for tri in terrain.findall(".//c:triangles", ns):
        assert tri.find("c:input[@semantic='NORMAL']", ns).get("offset") == "1"
        assert tri.find("c:input[@semantic='VERTEX']", ns).get("offset") == "0"
        packed = list(map(int, tri.find("c:p", ns).text.split()))
        assert len(packed) == int(tri.get("count"))*6
        indices = packed[::2]
        assert indices == packed[1::2], "Expected matching vertex and normal indices"
        assert len(indices) == int(tri.get("count"))*3
        assert min(indices) >= 0 and max(indices) < len(coords)//3
        triangles += len(indices)//3
    assert triangles == 180000
    # Every quad uses upward winding; the mesh is open at the world boundary.
    assert coords[0] == -150 and coords[-3] == 150
    rover = next(m for m in models if m.get("name") == "survey_rover")
    sensors = rover.findall(".//sensor")
    assert sorted(s.get("type") for s in sensors) == ["camera"]*3+["gpu_lidar","imu"]
    assert len(rover.findall("joint")) == 4
    assert rover.findtext(".//lidar/noise/type") == "gaussian"
    physics_world = ET.parse(ROOT / "assets/worlds/wildos_physics_only.sdf")
    assert not physics_world.findall(".//sensor")
    assert len(physics_world.findall(".//world/model")) == len(models)
    ET.parse(ROOT / "assets/models/wildos_terrain/model.sdf")
    ET.parse(ROOT / "assets/layout.svg")
    print(f"PASS: {len(models)-1} static models, {triangles} terrain triangles, 5 sensors, 4 tasks")
    print(f"PASS: {len(normals)//3} explicit unit vertex normals with validated triangle bindings")
    print("Not tested: Gazebo loading, rendering, contacts, sensor topics, ROS bridge or WildOS.")


if __name__ == "__main__":
    validate()
