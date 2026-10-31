"""Italy + native D-Series preview. No network services except localhost TCOM."""
import argparse
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import socket
import subprocess
import time
import uuid
import zipfile

from beamngpy import BeamNGpy, Scenario, Vehicle, ProceduralCube, ProceduralCylinder
from beamngpy.misc.quat import compute_rotation_matrix
from beamngpy.sensors import Camera, AdvancedIMU, Lidar
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parent
HOME = Path(r"D:\BEAMNG\BeamNG.tech.v0.39.4.0")
SPAWNS = "levels/italy/main/MissionGroup/spawnpoints/items.level.json"
ROAD_FILE = "levels/italy/main/MissionGroup/decalroads/asphalt/road_details/items.level.json"
ROAD_ID = "d638a2ec-6b50-44e9-a5a9-6445e17a4d8a"
CONFIG = "vehicles/pickup/d15_4wd_A.pc"
CONFIG_INFO = "vehicles/pickup/info_d15_4wd_A.json"
NAME = "nizhen_italy_dseries_preview"


def discover(home):
    evidence = {}
    def read(archive, member):
        raw = archive.read(member)
        evidence[member] = {"archive": str(archive.filename),
                            "sha256": hashlib.sha256(raw).hexdigest()}
        return raw.decode("utf-8-sig")

    with zipfile.ZipFile(home / "content/levels/italy.zip") as archive:
        spawn = next(o for o in map(json.loads, read(archive, SPAWNS).splitlines())
                     if o.get("name") == "spawn_crossroads")
        road = next(o for o in map(json.loads, read(archive, ROAD_FILE).splitlines())
                    if o.get("persistentId") == ROAD_ID)
    with zipfile.ZipFile(home / "content/vehicles/pickup.zip") as archive:
        info = json.loads(read(archive, CONFIG_INFO))
        config = read(archive, CONFIG)
        assert info["Drivetrain"] == "4WD" and info["Transmission"] == "Automatic"
        # Shipped legacy PC syntax is not strict JSON; let BeamNG parse the original.
        for part in ("pickup_transfer_case_4WD", "pickup_wheeldata_F", "pickup_wheeldata_R"):
            assert part in config, part
    matrix = [spawn["rotationMatrix"][i:i+3] for i in (0, 3, 6)]
    quat = Rotation.from_matrix(matrix).as_quat().tolist()
    assert max(abs(a-b) for a, b in zip(compute_rotation_matrix(quat).ravel(),
                                      spawn["rotationMatrix"])) < 1e-6
    # These are actual decal-road surface nodes, not guessed world coordinates.
    # Keep the entire vehicle spawn area clear; objects occupy only a short strip.
    a, b = road["nodes"]
    objects = []
    for kind, fraction, size in (("cube", 0.65, [0.7, 0.7, 0.5]),
                                  ("cylinder", 1.0, [0.3, 1.5])):
        ground = [a[i] + fraction * (b[i] - a[i]) for i in range(3)]
        clearance = math.dist(ground[:2], spawn["position"][:2])
        assert clearance > 15, "Obstacle too close to spawn"
        height = size[-1]
        objects.append({"kind": kind, "name": f"nz_{kind}", "size": size,
                        "pos": [ground[0], ground[1], ground[2] + height/2],
                        "road_node_fraction": fraction, "spawn_clearance_m": clearance})
    return {"level": "italy", "terrain": "Existing BeamNG Italy, not a new terrain",
            "model": "pickup", "part_config": CONFIG, "vehicle_metadata": info,
            "spawn": spawn, "rot_quat": quat, "road": road, "objects": objects,
            "evidence": evidence, "beamngpy": "1.36",
            "surface_note": "Existing asphalt at preview spawn; no physical cobblestones added"}


def build(data, name=NAME, objects=True):
    scenario = Scenario("italy", name, description="Native Italy rural preview with D15 4WD",
                        authors="Local workspace")
    car = Vehicle("nz_ego", model="pickup", part_config=data["part_config"])
    scenario.add_vehicle(car, pos=tuple(data["spawn"]["position"]),
                         rot_quat=tuple(data["rot_quat"]))
    if objects:
        for obj in data["objects"]:
            if obj["kind"] == "cube":
                mesh = ProceduralCube(tuple(obj["pos"]), tuple(obj["size"]), obj["name"])
            else:
                mesh = ProceduralCylinder(tuple(obj["pos"]), obj["size"][0],
                                          obj["size"][1], obj["name"])
            scenario.add_procedural_mesh(mesh)
    return scenario, car


class OfflineWriter:
    """Capture Scenario.make's native payload, mirroring shipped CreateScenario."""
    def _message(self, message, **payload):
        assert message == "CreateScenario" and payload["json"] is True
        folder = ROOT / "generated" / "levels" / payload["level"] / "scenarios" / payload["name"]
        folder.mkdir(parents=True, exist_ok=True)
        base = folder / payload["name"]
        base.with_suffix(".prefab.json").write_text(payload["prefab"], encoding="utf-8")
        base.with_suffix(".json").write_text(json.dumps([payload["info"]], indent=2), encoding="utf-8")
        return str(base.with_suffix(".json"))


def sensor_specs(args):
    specs = []
    if not args.no_sensors:
        specs.append((Camera, "front", dict(pos=(0, -1.5, 2), dir=(0, -1, 0))))
        if args.side_cameras or args.full_sensors:
            specs.extend([(Camera, "left", dict(pos=(-1.2, 0, 2), dir=(-1, 0, 0))),
                          (Camera, "right", dict(pos=(1.2, 0, 2), dir=(1, 0, 0)))])
        for _, _, options in specs:
            options.update(resolution=(640, 360), requested_update_time=0.1,
                           is_render_annotations=False, is_render_depth=False,
                           is_using_shared_memory=False, is_visualised=False)
        specs.append((AdvancedIMU, "imu", dict(pos=(0, 0, 1), gfx_update_time=0.1,
                      physics_update_time=0.01, is_allow_wheel_nodes=False, is_visualised=False)))
        if args.lidar or args.full_sensors:
            specs.append((Lidar, "lidar", dict(pos=(0, 0, 2.2), requested_update_time=0.2,
                          vertical_resolution=16, frequency=10, max_distance=60,
                          is_using_shared_memory=False, is_visualised=False)))
    return specs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--home", type=Path, default=HOME)
    parser.add_argument("--port", type=int, default=64256)
    parser.add_argument("--offline", action="store_true", help="Generate native files without contacting simulator")
    parser.add_argument("--launch", action="store_true", help="Launch only if no BeamNG process is running")
    parser.add_argument("--replace-current", action="store_true", help="Explicit permission to load this scenario")
    parser.add_argument("--full-sensors", action="store_true")
    parser.add_argument("--side-cameras", action="store_true")
    parser.add_argument("--lidar", action="store_true")
    parser.add_argument("--no-sensors", action="store_true")
    parser.add_argument("--no-objects", action="store_true")
    parser.add_argument("--seconds", type=float, default=0, help="Capture duration; 0 waits for Ctrl+C")
    args = parser.parse_args()
    if importlib.metadata.version("beamngpy") != "1.36":
        parser.error("Use the dedicated venv with beamngpy==1.36")
    if args.seconds < 0 or not math.isfinite(args.seconds):
        parser.error("--seconds must be finite and nonnegative")
    if args.no_sensors and (args.full_sensors or args.side_cameras or args.lidar):
        parser.error("--no-sensors conflicts with requested sensors")
    if not args.offline and not args.replace_current:
        parser.error("No world changes made. Supply --replace-current to load, or --offline to generate only.")
    data = discover(args.home)
    generated = ROOT / "generated"
    generated.mkdir(exist_ok=True)
    
    (generated / "provenance.json").write_text(json.dumps(data, indent=2), encoding="utf-8")
    preview, _ = build(data, objects=not args.no_objects)
    preview.make(OfflineWriter())
    (generated / "runtime_objects.json").write_text(json.dumps(
        [] if args.no_objects else data["objects"], indent=2), encoding="utf-8")
    print("Native vehicle prefab and provenance generated:", generated)
    if args.offline:
        return
    if args.launch:
        processes = subprocess.run(["tasklist", "/FO", "CSV", "/NH"], capture_output=True, check=True)
        if b"beamng" in processes.stdout.lower():
            parser.error("BeamNG process already running. Connect without --launch; do not start a second instance.")
    else:
        try:
            with socket.create_connection(("127.0.0.1", args.port), timeout=2):
                pass
        except OSError:
            parser.error(f"No localhost TCOM listener at {args.port}. Start BeamNG with -tcom {args.port}, or use --launch after exiting it yourself.")
    bng = BeamNGpy("127.0.0.1", args.port, home=str(args.home), quit_on_close=False, socket_timeout=180)
    sensors = []
    # A new namespace per load never overwrites previous scenario files or editor work.
    scenario, car = build(data, NAME + "_" + uuid.uuid4().hex[:10], not args.no_objects)
    output = ROOT / "captures" / scenario.name
    try:
        bng.open(launch=args.launch)
        scenario.make(bng)
        bng.scenario.load(scenario)
        bng.scenario.start()
        car.control(throttle=0, brake=1, parkingbrake=1)
        output.mkdir(parents=True)
        (output / "scenario_path.txt").write_text(str(scenario.path), encoding="utf-8")
        for cls, name, options in sensor_specs(args):
            sensors.append((name, cls("nz_" + name, bng, car, **options)))
            time.sleep(0.5)
        print("Scene loaded; parking brake engaged. No autonomous driving. Captures:", output)
        start = time.monotonic()
        while args.seconds == 0 or time.monotonic() - start < args.seconds:
            for name, sensor in sensors:
                reading = sensor.poll()
                if isinstance(sensor, Camera):
                    reading["colour"].save(output / f"{name}.png")
                elif isinstance(sensor, Lidar):
                    import numpy as np
                    np.save(output / "lidar.npy", reading["pointCloud"])
                else:
                    (output / "imu.json").write_text(json.dumps(reading, indent=2), encoding="utf-8")
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("Capture stopped. Simulator remains open.")
    finally:
        for name, sensor in reversed(sensors):
            try:
                sensor.remove()
            except Exception as exc:
                print(f"Could not remove sensor {name}: {exc}")
        bng.disconnect()


if __name__ == "__main__":
    main()
