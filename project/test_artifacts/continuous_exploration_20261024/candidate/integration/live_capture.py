"""Live multi-view capture with a vehicle-mounted sensor rig.

Camera, AdvancedIMU and LiDAR are attached to the ego vehicle (is_static=False,
is_dir_world_space=False). The ego is teleported through the viewpoint list; at
each pose the simulation is settled, frozen with a single step, and image / cloud /
IMU / pose are captured under one host-monotonic stamp.

The mounted LiDAR cloud is normalised to world coordinates (the fixture ground
plane at z=0.1 decides whether raw points are already world or sensor-local), so
the existing object_search pipeline consumes it unchanged.

Run from the vision_query root (plain script):
    python -B integration/live_capture.py --home ... --user ... --replace-current \
        --poses="-4,1.5;0,1.0;4,1.5" --output ~/capture_out
"""
import argparse
import hashlib
import json
import platform
import time
import uuid
from pathlib import Path

import numpy as np

import capture_frames as cf

SENSOR_CAL_ROOT = Path(__file__).resolve().parent.parent / "sensor_calibration"
TIME_SOURCE = "beamng_host_monotonic_paused"
UP_WORLD = (0.0, 0.0, 1.0)
CAR_Z = 0.50
CAM_POS = (0.0, -1.5, 2.35)
CAM_DIR = (0.0, -1.0, 0.0)
IMU_POS = (0.0, 0.0, 1.0)
# LiDAR on a high mast, tilted down, full 360 so the ring around the car is covered.
LIDAR_POS = (0.0, 0.0, 3.6)
LIDAR_DIR = (0.0, -1.0, -0.45)
LIDAR_VRES = 64
LIDAR_VANGLE = 90.0
LIDAR_DENSITY = 20
LIDAR_MAX_DIST = 80.0
DEFAULT_POSES = "-4,1.5;0,1.0;4,1.5"


def parse_poses(text):
    poses = []
    for chunk in text.split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        x, y = chunk.split(",")
        poses.append((float(x), float(y)))
    if not poses:
        raise ValueError("empty pose list")
    return poses


def to_jsonable(value):
    if isinstance(value, dict):
        return {str(k): to_jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_jsonable(v) for v in value]
    if hasattr(value, "tolist"):
        return value.tolist()
    if isinstance(value, (int, float, str, bool)) or value is None:
        return value
    return str(value)


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--home", type=Path, required=True)
    p.add_argument("--user", type=Path, required=True)
    p.add_argument("--replace-current", action="store_true")
    p.add_argument("--poses", default=DEFAULT_POSES,
                   help="Semicolon list of car (x,y) viewpoints; car faces -Y")
    p.add_argument("--port", type=int, default=64256)
    p.add_argument("--timeout", type=float, default=60)
    p.add_argument("--gfx", choices=("dx11", "vk"))
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args(argv)
    if not a.replace_current:
        p.error("--replace-current is required")
    a.pose_list = parse_poses(a.poses)
    return a


def make_rig(bng, car):
    from beamngpy.sensors import AdvancedIMU, Camera, Lidar
    cam = Camera("nzs_live_cam", bng, car, pos=CAM_POS, dir=CAM_DIR, up=UP_WORLD,
                 is_static=False, is_dir_world_space=False, resolution=(cf.WIDTH, cf.HEIGHT),
                 field_of_view_y=60, near_far_planes=(0.05, 60), requested_update_time=-1,
                 is_using_shared_memory=False, is_visualised=False,
                 is_render_colours=True, is_render_depth=False, is_render_annotations=False)
    lidar = Lidar("nzs_live_lidar", bng, car, pos=LIDAR_POS, dir=LIDAR_DIR, up=UP_WORLD,
                  is_static=False, is_dir_world_space=False, requested_update_time=-1,
                  vertical_resolution=LIDAR_VRES, vertical_angle=LIDAR_VANGLE,
                  horizontal_angle=360, max_distance=LIDAR_MAX_DIST, density=LIDAR_DENSITY,
                  is_360_mode=True, is_using_shared_memory=False, is_visualised=False, is_annotated=False)
    imu = AdvancedIMU("nzs_live_imu", bng, car, pos=IMU_POS, dir=CAM_DIR, up=UP_WORLD,
                      is_using_gravity=True, physics_update_time=0.01, gfx_update_time=0.2,
                      is_send_immediately=False, is_dir_world_space=False, is_visualised=False)
    return {"camera": cam, "lidar": lidar, "imu": imu}


def ground_hits(points):
    board = (np.abs(points[:, 2] - 0.1) < 0.05) & (np.abs(points[:, 0]) < 11.8)
    board &= (points[:, 1] > -17.8) & (points[:, 1] < 11.8)
    return int(board.sum())


def cloud_to_world(cloud, lidar_position):
    """Pick raw-world vs translated-local (world axes) using the known ground plane."""
    raw = ground_hits(cloud)
    shifted = ground_hits(cloud + np.asarray(lidar_position, float))
    if raw >= shifted:
        return cloud, "raw_world"
    return cloud + np.asarray(lidar_position, float), "translated_local_world_axes"


def settle(bng, car, timeout, speed_tol=0.02, need=5):
    bng.control.resume()
    deadline = time.monotonic() + timeout
    count = 0
    while time.monotonic() < deadline:
        car.sensors.poll()
        speed = float(np.linalg.norm(dict(car.state)["vel"]))
        if speed < speed_tol:
            count += 1
            if count >= need:
                return True
        else:
            count = 0
        time.sleep(0.4)
    return False


def run(a):
    import sys
    sys.path.insert(0, str(SENSOR_CAL_ROOT))
    import importlib.metadata
    from fixture import ROOT, ART, MOD, scenario
    from install import user_paths, verified_mod
    from inspect_local import inspect_home
    from build import write_json

    if importlib.metadata.version("beamngpy") != "1.36":
        raise ValueError("Requires beamngpy==1.36")
    inspect_home(a.home)
    parent, effective = user_paths(a.user)
    if (effective / "mods" / MOD).read_bytes() != verified_mod().read_bytes():
        raise ValueError("Own installed ZIP differs")

    from beamngpy import BeamNGpy
    from beamngpy.connection import Connection

    binary = "Bin64/BeamNG.tech.x64.exe" if platform.system() == "Windows" else "BinLinux/BeamNG.tech.x64"
    bng = BeamNGpy("127.0.0.1", a.port, home=str(a.home.resolve()), user=str(parent), binary=binary,
                   quit_on_close=False, socket_timeout=a.timeout, gfx=a.gfx)

    output = a.output.resolve() / uuid.uuid4().hex[:8]
    frames_dir = output / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)
    epoch = output.name
    cf.progress("Output: " + str(output))

    report = {"schema": "live-capture/1", "epoch": epoch, "poses": [list(p) for p in a.pose_list],
              "rig": {"camera": list(CAM_POS), "lidar": list(LIDAR_POS), "imu": list(IMU_POS)},
              "time_source": TIME_SOURCE, "frames": [],
              "scope": "live teleported multi-view capture with mounted camera/imu/lidar"}
    rig, car = {}, None
    try:
        cf.progress("Connecting to BeamNG")
        bng.open(launch=True, listen_ip="127.0.0.1")
        actual = bng.control.queue_lua_command("return FS:getUserPath()", response=True)
        report["actual_userpath"] = actual
        if not isinstance(actual, str) or Path(actual).resolve() != effective:
            raise ValueError("Actual FS:getUserPath differs; world untouched")

        manifest = json.loads((ROOT / "generated/fixture.json").read_text(encoding="utf-8"))
        scene, car = scenario(manifest, "nzs_live_" + uuid.uuid4().hex)
        scene.make(bng)
        bng.scenario.load(scene, connect_player_vehicle=False, connect_existing_vehicles=False)
        endpoint = bng.vehicles.start_connection(car, car.extensions)
        if endpoint["vid"] != car.vid:
            raise RuntimeError("Unexpected vehicle connection ID")
        car.connection = Connection("127.0.0.1", int(endpoint["result"]), timeout=a.timeout)
        car.connect(bng)
        bng.scenario.start()
        car.ai.set_mode("disabled")
        car.set_shift_mode("realistic_automatic")
        car.control(throttle=0, brake=1, parkingbrake=1, steering=0)
        bng.control.resume()
        time.sleep(2.0)

        cf.progress("Attaching mounted rig (camera/imu/lidar)")
        rig = make_rig(bng, car)
        time.sleep(1.0)

        for i, (px, py) in enumerate(a.pose_list):
            cf.progress("Pose %d/%d -> car (%.2f, %.2f)" % (i + 1, len(a.pose_list), px, py))
            bng.control.resume()
            car.teleport((px, py, CAR_Z), reset=True)
            if not settle(bng, car, a.timeout):
                raise RuntimeError("vehicle did not settle at pose %d" % i)
            car.sensors.poll()
            cf.progress("  car at %s" % [round(v, 3) for v in dict(car.state)["pos"]])

            bng.control.pause()
            bng.control.step(1)
            state_before = dict(car.state)
            poses_before = {"camera": cf.sensor_pose(rig["camera"]), "lidar": cf.sensor_pose(rig["lidar"])}
            stamp_ns = int(round(time.monotonic() * 1e9))

            image_reading, _, _ = cf.adhoc(rig["camera"], a.timeout)
            image = image_reading.get("colour")
            if image is None or image.size != (cf.WIDTH, cf.HEIGHT):
                raise RuntimeError("Missing/incorrect RGB image at pose %d" % i)
            frame_dir = frames_dir / ("frame_%03d" % i)
            frame_dir.mkdir(parents=True, exist_ok=True)
            image_path = frame_dir / "image.png"
            image.save(image_path)
            image_sha = hashlib.sha256(image_path.read_bytes()).hexdigest()

            cloud_reading, _, _ = cf.adhoc(rig["lidar"], a.timeout)
            cloud = np.asarray(cloud_reading.get("pointCloud"), dtype=np.float64)
            if cloud.ndim != 2 or cloud.shape[1] != 3 or not np.isfinite(cloud).all():
                raise RuntimeError("Malformed LiDAR cloud at pose %d" % i)
            world_cloud, frame_observed = cloud_to_world(cloud, poses_before["lidar"][0])
            np.save(frame_dir / "cloud.npy", world_cloud, allow_pickle=False)

            try:
                imu_reading = rig["imu"].poll()
            except Exception as exc:
                imu_reading = {"poll_error": str(exc)}
            write_json(frame_dir / "imu.json", {"schema": "imu-sample/1", "epoch": epoch,
                                                "stamp_ns": stamp_ns, "records": to_jsonable(imu_reading)})

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
                "imu": {"file": "imu.json"},
                "camera": {"position": cam_pos.tolist(), "direction": cam_dir.tolist(), "up": list(UP_WORLD),
                           "frame": "vehicle_mounted"},
                "pose": {"source_frame": "smallgrid_world", "target_frame": "static_optical",
                         "R": rotation.tolist(), "translation_m": translation.tolist(), "units": "m"},
                "timing": {"stamp_ns": stamp_ns, "source": TIME_SOURCE, "epoch": epoch, "time_kind": "acquisition"},
                "pose_stability": stability,
                "notes": ["mounted camera/imu/lidar; teleported viewpoint; paused then single-stepped"],
                "car_position": state_before["pos"],
            }
            write_json(frame_dir / "capture.json", record)
            report["frames"].append({"frame_index": i, "car_xy": [px, py], "stable": stable,
                                     "cloud_frame": frame_observed, "path": str(frame_dir),
                                     "points": int(len(world_cloud))})
            cf.progress("  frame %d stable=%s cloud=%s points=%d" % (i, stable, frame_observed, len(world_cloud)))
            bng.control.resume()

        report["passed"] = bool(report["frames"]) and all(f["stable"] for f in report["frames"])
        return 0 if report["passed"] else 2
    finally:
        for sensor in reversed(list(rig.values())):
            try:
                sensor.remove()
            except Exception as exc:
                report.setdefault("cleanup_errors", []).append(str(exc))
        write_json(output / "capture_report.json", report)
        cf.progress("Report: " + str(output / "capture_report.json"))
        bng.disconnect()


if __name__ == "__main__":
    raise SystemExit(run(parse_args()))
