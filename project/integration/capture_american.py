"""American Road capture adapter for the basic WildOS flow (no ExploRFM heads).

Reads the real SpawnSphere transform from the map archive, loads the level with
the mounted camera + LiDAR rig (960x640 RGB, FOV 60 so integration nominal K
applies), teleports the ego along its forward axis for a small multi-view
baseline, and writes integration capture-frame/1 records (image, world cloud,
camera pose, IMU). Downstream: vocabulary_query/query.py + object_search.

    python -B integration/capture_american.py --home ... --user ... \
        --map-zip ~/beamng-maps/american_road_0.39.zip --spawn spawns_road1 \
        --poses="-4,0,4" --output ~/capture_out
"""
import argparse
import hashlib
import json
import platform
import time
import uuid
import zipfile
from pathlib import Path

import numpy as np

import capture_frames as cf

LEVEL = "american_road"
PART_CONFIG = "vehicles/pickup/d15_4wd_A.pc"
TIME_SOURCE = "beamng_host_monotonic_paused"
UP_WORLD = (0.0, 0.0, 1.0)
CAM_POS = (0.0, -1.5, 2.35)
IMU_POS = (0.0, 0.0, 1.0)
LIDAR_POS = (0.0, 0.0, 3.6)
LIDAR_DIR = (0.0, -1.0, -0.45)


def parse_floats(text):
    return [float(x) for x in text.split(",") if x.strip() != ""]


def read_spawn(map_zip, spawn_name):
    """Line-wise Torque JSONL parse of items.level.json for the SpawnSphere."""
    with zipfile.ZipFile(map_zip) as archive:
        for member in archive.infolist():
            if not (member.filename.startswith("levels/%s/" % LEVEL)
                    and member.filename.endswith("items.level.json")):
                continue
            text = archive.read(member).decode("utf-8-sig")
            if spawn_name not in text:
                continue
            for line in text.splitlines():
                line = line.strip()
                if not line.startswith("{"):
                    continue
                try:
                    record = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if not isinstance(record, dict) or record.get("name") != spawn_name:
                    continue
                if record.get("class") != "SpawnSphere":
                    raise ValueError("%s is not a SpawnSphere" % spawn_name)
                position = [float(v) for v in record["position"]]
                matrix = np.asarray(record.get("rotationMatrix", np.eye(3).reshape(-1)), float).reshape(3, 3)
                from scipy.spatial.transform import Rotation
                quat = Rotation.from_matrix(matrix).as_quat().tolist()
                return {"position": position, "rot_quat": quat, "rotationMatrix": matrix.reshape(-1).tolist()}
    raise ValueError("spawn %s not found in %s" % (spawn_name, map_zip))


def forward_of(quat):
    from scipy.spatial.transform import Rotation
    return Rotation.from_quat(quat).apply([0.0, -1.0, 0.0])


def cloud_to_world(cloud, lidar_position):
    """American Road has large world magnitudes, so a magnitude test is decisive."""
    if cloud.shape[0] and float(np.median(np.abs(cloud))) > 200.0:
        return cloud, "raw_world"
    return cloud + np.asarray(lidar_position, float), "translated_local_world_axes"


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--home", type=Path, required=True)
    p.add_argument("--user", type=Path, required=True)
    p.add_argument("--map-zip", type=Path, required=True)
    p.add_argument("--spawn", default="spawns_road1")
    p.add_argument("--poses", default="-4,0,4",
                   help="Comma list of forward offsets (metres) from the spawn along the vehicle forward axis")
    p.add_argument("--replace-current", action="store_true")
    p.add_argument("--port", type=int, default=64256)
    p.add_argument("--timeout", type=float, default=600)
    p.add_argument("--gfx", choices=("dx11", "vk"))
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args(argv)
    if not a.replace_current:
        p.error("--replace-current is required")
    a.offsets = parse_floats(a.poses)
    return a


def make_rig(bng, car, resolution=(640, 360), lidar_resolution=32,
             lidar_max_distance=60, lidar_density=10):
    from beamngpy.sensors import AdvancedIMU, Camera, Lidar
    cam = Camera("am_cam", bng, car, pos=CAM_POS, dir=(0.0, -1.0, 0.0), up=UP_WORLD,
                 is_static=False, is_dir_world_space=False, resolution=resolution,
                 field_of_view_y=60, near_far_planes=(0.05, 200), requested_update_time=-1,
                 is_using_shared_memory=False, is_visualised=False,
                 is_render_colours=True, is_render_depth=False, is_render_annotations=False)
    lidar = Lidar("am_lidar", bng, car, pos=LIDAR_POS, dir=LIDAR_DIR, up=UP_WORLD,
                  is_static=False, is_dir_world_space=False, requested_update_time=-1,
                   vertical_resolution=lidar_resolution, vertical_angle=90, horizontal_angle=360,
                   max_distance=lidar_max_distance, density=lidar_density, is_360_mode=True,
                  is_using_shared_memory=False, is_visualised=False, is_annotated=False)
    imu = AdvancedIMU("am_imu", bng, car, pos=IMU_POS, dir=(0.0, -1.0, 0.0), up=UP_WORLD,
                      is_using_gravity=True, physics_update_time=0.01, gfx_update_time=0.2,
                      is_send_immediately=False, is_dir_world_space=False, is_visualised=False)
    return {"camera": cam, "lidar": lidar, "imu": imu}


def settle(bng, car, timeout, speed_tol=0.05, need=5):
    bng.control.resume()
    deadline = time.monotonic() + timeout
    count = 0
    while time.monotonic() < deadline:
        car.sensors.poll()
        if float(np.linalg.norm(dict(car.state)["vel"])) < speed_tol:
            count += 1
            if count >= need:
                return True
        else:
            count = 0
        time.sleep(0.4)
    return False


def run(a):
    from beamngpy import BeamNGpy, Scenario, Vehicle
    from beamngpy.connection import Connection

    spawn = read_spawn(a.map_zip.expanduser().resolve(), a.spawn)
    forward = forward_of(spawn["rot_quat"])
    cf.progress("spawn %s pos=%s forward=%s" % (a.spawn, spawn["position"], forward.round(3).tolist()))

    binary = "Bin64/BeamNG.tech.x64.exe" if platform.system() == "Windows" else "BinLinux/BeamNG.tech.x64"
    bng = BeamNGpy("127.0.0.1", a.port, home=str(a.home.resolve()), user=str(a.user.resolve()),
                   binary=binary, quit_on_close=False, socket_timeout=a.timeout, gfx=a.gfx)

    output = a.output.resolve() / uuid.uuid4().hex[:8]
    frames_dir = output / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)
    epoch = output.name
    cf.progress("Output: " + str(output))
    report = {"schema": "capture-american/1", "epoch": epoch, "level": LEVEL, "spawn": a.spawn,
              "spawn_transform": spawn, "offsets": a.offsets, "frames": [],
              "time_source": TIME_SOURCE, "scope": "American Road basic WildOS capture; no heads"}
    rig, car = {}, None
    try:
        cf.progress("Connecting/launching BeamNG")
        bng.open(launch=True, listen_ip="127.0.0.1")
        scene = Scenario(LEVEL, "am_" + uuid.uuid4().hex, description="WildOS basic flow")
        car = Vehicle("am_ego", model="pickup", part_config=PART_CONFIG)
        scene.add_vehicle(car, pos=tuple(spawn["position"]), rot_quat=tuple(spawn["rot_quat"]),
                          safe_spawn=False)
        scene.make(bng)
        bng.scenario.load(scene, connect_player_vehicle=False, connect_existing_vehicles=False)
        endpoint = bng.vehicles.start_connection(car, car.extensions)
        car.connection = Connection("127.0.0.1", int(endpoint["result"]), timeout=a.timeout)
        car.connect(bng)
        bng.scenario.start()
        car.ai.set_mode("disabled")
        car.set_shift_mode("realistic_automatic")
        car.control(throttle=0, brake=1, parkingbrake=1, steering=0, gear=0)
        bng.control.resume()
        time.sleep(3.0)
        cf.progress("Attaching mounted rig")
        rig = make_rig(bng, car)
        time.sleep(1.0)

        base = np.asarray(spawn["position"], float)
        for i, offset in enumerate(a.offsets):
            target = base + forward * offset
            cf.progress("View %d/%d offset=%.2f -> %s" % (i + 1, len(a.offsets), offset,
                                                          target.round(2).tolist()))
            car.teleport(tuple(target), rot_quat=tuple(spawn["rot_quat"]), reset=True)
            if not settle(bng, car, min(a.timeout, 90)):
                cf.progress("  warning: not fully settled at view %d" % i)
            bng.control.pause()
            bng.control.step(1)
            state_before = dict(car.state)
            poses_before = {"camera": cf.sensor_pose(rig["camera"]), "lidar": cf.sensor_pose(rig["lidar"])}
            stamp_ns = int(round(time.monotonic() * 1e9))

            image_reading, _, _ = cf.adhoc(rig["camera"], a.timeout)
            image = image_reading.get("colour")
            if image is None or image.size != (cf.WIDTH, cf.HEIGHT):
                raise RuntimeError("Missing/incorrect RGB image at view %d" % i)
            frame_dir = frames_dir / ("frame_%03d" % i)
            frame_dir.mkdir(parents=True, exist_ok=True)
            image_path = frame_dir / "image.png"
            image.save(image_path)
            image_sha = hashlib.sha256(image_path.read_bytes()).hexdigest()

            cloud_reading, _, _ = cf.adhoc(rig["lidar"], a.timeout)
            cloud = np.asarray(cloud_reading.get("pointCloud"), dtype=np.float64)
            if cloud.ndim != 2 or cloud.shape[1] != 3 or not np.isfinite(cloud).all():
                raise RuntimeError("Malformed LiDAR cloud at view %d" % i)
            world_cloud, frame_observed = cloud_to_world(cloud, poses_before["lidar"][0])
            np.save(frame_dir / "cloud.npy", world_cloud, allow_pickle=False)

            state_after = dict(car.state)
            poses_after = {"camera": cf.sensor_pose(rig["camera"]), "lidar": cf.sensor_pose(rig["lidar"])}
            stable, stability = cf.pose_stability(state_before, state_after, poses_before, poses_after)
            cam_pos, cam_dir = poses_before["camera"]
            rotation, translation = cf.world_to_camera(cam_pos, cam_dir)
            record = {
                "schema": "capture-frame/1", "epoch": epoch, "frame_index": i,
                "synchronized": bool(stable), "time_source": TIME_SOURCE,
                "image": {"file": "image.png", "sha256": image_sha, "width": cf.WIDTH, "height": cf.HEIGHT},
                "cloud": {"file": "cloud.npy", "coordinates": "world", "units": "m",
                          "points": int(len(world_cloud)), "frame_observed": frame_observed},
                "camera": {"position": cam_pos.tolist(), "direction": cam_dir.tolist(), "up": list(UP_WORLD),
                           "frame": "vehicle_mounted"},
                "pose": {"source_frame": "american_road_world", "target_frame": "static_optical",
                         "R": rotation.tolist(), "translation_m": translation.tolist(), "units": "m"},
                "timing": {"stamp_ns": stamp_ns, "source": TIME_SOURCE, "epoch": epoch, "time_kind": "acquisition"},
                "pose_stability": stability,
                "notes": ["American Road; teleported view; paused then single-stepped"],
                "car_position": state_before["pos"],
            }
            (frame_dir / "capture.json").write_text(json.dumps(record, indent=2), encoding="utf-8")
            report["frames"].append({"frame_index": i, "offset": offset, "stable": bool(stable),
                                     "path": str(frame_dir), "points": int(len(world_cloud)),
                                     "cloud_frame": frame_observed})
            cf.progress("  frame %d stable=%s cloud=%s points=%d" % (i, stable, frame_observed, len(world_cloud)))
            bng.control.resume()

        report["passed"] = bool(report["frames"])
        return 0 if report["passed"] else 2
    finally:
        for sensor in reversed(list(rig.values())):
            try:
                sensor.remove()
            except Exception as exc:
                report.setdefault("cleanup_errors", []).append(str(exc))
        (output / "capture_report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        cf.progress("Report: " + str(output / "capture_report.json"))
        bng.disconnect()


if __name__ == "__main__":
    raise SystemExit(run(parse_args()))
