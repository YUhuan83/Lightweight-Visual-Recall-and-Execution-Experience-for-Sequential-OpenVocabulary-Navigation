"""Capture synchronized stationary frames from the calibration fixture scene.

Synchronization strategy: pause the simulation, advance exactly one step, then
request the camera and the LiDAR back-to-back while the world is frozen again.
The physics state and both sensor poses are re-read after the requests; if any of
them moved, the frame is marked unsynchronized instead of being silently trusted.
One shared acquisition stamp is assigned to image, cloud and pose.

Requires: --home (BeamNG home), --user (userpath parent), --replace-current.
Optional: --frames, --stride, --timeout, --port, --gfx, --output.

Run as a plain script (it has no relative imports):
    python integration/capture_frames.py --home ... --user ... --replace-current --output ...
"""
import argparse
import hashlib
import json
import platform
import time
import uuid
from pathlib import Path

import numpy as np

SENSOR_CAL_ROOT = Path(__file__).resolve().parent.parent / "sensor_calibration"
TIME_SOURCE = "beamng_host_monotonic_paused"
UP_WORLD = (0.0, 0.0, 1.0)
FIXTURE_CAMERA_POS = (0.0, 0.0, 2.35)
FIXTURE_CAMERA_DIR = (0.0, -1.0, 0.0)
WIDTH, HEIGHT = 960, 640


def progress(message):
    print(time.strftime("[%H:%M:%S] ") + message, flush=True)


def optical_basis(forward, up_world=UP_WORLD):
    """Camera axes (right, down, forward) in world, matching fixture.coordinates().

    fixture.coordinates() uses right = forward x up and down = forward x right.
    """
    f = np.asarray(forward, float)
    f = f / np.linalg.norm(f)
    right = np.cross(f, np.asarray(up_world, float))
    right = right / np.linalg.norm(right)
    down = np.cross(f, right)
    return right, down, f, np.stack((right, down, f))


def world_to_camera(position, direction, up_world=UP_WORLD):
    """World->camera SE3 (R, t) with p_cam = R @ p_world + t, axes x_right y_down z_forward."""
    rotation, _, _, stacked = optical_basis(direction, up_world)
    translation = -stacked @ np.asarray(position, float)
    return stacked, translation


def pose_stability(before_state, after_state, before_poses, after_poses, position_tol=1e-4, direction_tol=1e-4, speed_tol=1e-2):
    """Compare frozen-sim snapshots; return (stable, evidence)."""
    speed = float(np.linalg.norm(after_state.get("vel", [0.0, 0.0, 0.0])))
    position_delta = float(np.linalg.norm(np.asarray(after_state["pos"]) - np.asarray(before_state["pos"])))
    up_delta = 0.0
    if "up" in before_state and "up" in after_state:
        up_delta = float(np.linalg.norm(np.asarray(after_state["up"]) - np.asarray(before_state["up"])))
    sensor_delta = 0.0
    for name in before_poses:
        sensor_delta = max(sensor_delta, float(np.linalg.norm(
            np.asarray(after_poses[name][0]) - np.asarray(before_poses[name][0]))),
            float(np.linalg.norm(np.asarray(after_poses[name][1]) - np.asarray(before_poses[name][1]))))
    stable = position_delta <= position_tol and up_delta <= direction_tol and sensor_delta <= direction_tol and speed < speed_tol
    return stable, {"vehicle_position_delta_m": position_delta, "vehicle_up_delta": up_delta,
                    "sensor_pose_delta": sensor_delta, "vehicle_speed": speed}


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--home", type=Path, required=True)
    p.add_argument("--user", type=Path, required=True, help="Userpath parent or its current directory")
    p.add_argument("--replace-current", action="store_true")
    p.add_argument("--frames", type=int, default=1)
    p.add_argument("--stride", type=float, default=0.0,
                   help="Lateral metres between frames; 0 keeps one viewpoint")
    p.add_argument("--port", type=int, default=64256)
    p.add_argument("--timeout", type=float, default=60)
    p.add_argument("--gfx", choices=("dx11", "vk"))
    p.add_argument("--output", type=Path, required=True, help="Base output directory; run ID subfolder created")
    a = p.parse_args(argv)
    if not a.replace_current:
        p.error("--replace-current is required")
    if a.frames < 1:
        p.error("--frames must be >= 1")
    if a.stride < 0:
        p.error("--stride must be >= 0")
    return a


def adhoc(sensor, timeout):
    """Single ad-hoc sample with host-monotonic bracketing; no invented simulation time."""
    deadline = time.monotonic() + timeout
    before = dict(position=sensor.get_position(), direction=sensor.get_direction())
    start = time.monotonic()
    request = sensor.send_ad_hoc_poll_request()
    while not sensor.is_ad_hoc_poll_request_ready(request):
        if time.monotonic() - start > timeout or time.monotonic() > deadline:
            raise TimeoutError("Ad-hoc sensor request timed out for " + getattr(sensor, "name", "sensor"))
        time.sleep(0.05)
    reading = sensor.collect_ad_hoc_poll_request(request)
    return reading, before, dict(position=sensor.get_position(), direction=sensor.get_direction())


def sensor_pose(sensor):
    return np.asarray(sensor.get_position(), float), np.asarray(sensor.get_direction(), float)


def make_sensors(bng, car, offset, timeout):
    from beamngpy.sensors import Camera, Lidar
    pos = (FIXTURE_CAMERA_POS[0] + offset, FIXTURE_CAMERA_POS[1], FIXTURE_CAMERA_POS[2])
    cam = Camera("nzs_cap_cam", bng, None, pos=pos, dir=FIXTURE_CAMERA_DIR, up=UP_WORLD,
                 is_static=True, is_dir_world_space=True, resolution=(WIDTH, HEIGHT),
                 field_of_view_y=60, near_far_planes=(0.05, 60), requested_update_time=-1,
                 is_using_shared_memory=False, is_visualised=False,
                 is_render_colours=True, is_render_depth=False, is_render_annotations=False)
    lidar = Lidar("nzs_cap_lidar", bng, None, pos=pos, dir=FIXTURE_CAMERA_DIR, up=UP_WORLD,
                  is_static=True, is_dir_world_space=True, requested_update_time=-1,
                  vertical_resolution=128, vertical_angle=60, horizontal_angle=120,
                  max_distance=60, density=1,
                  is_using_shared_memory=False, is_visualised=False, is_annotated=False)
    return cam, lidar


def capture_frames(a):
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
    progress("Output: " + str(output))

    report = {"schema": "capture-run/1", "epoch": epoch, "frame_count": a.frames,
              "time_source": TIME_SOURCE, "frames": [], "scope": "fixture stationary capture; not navigation"}
    sensors, car = [], None
    try:
        progress("Connecting to BeamNG")
        bng.open(launch=True, listen_ip="127.0.0.1")
        actual = bng.control.queue_lua_command("return FS:getUserPath()", response=True)
        report["actual_userpath"] = actual
        if not isinstance(actual, str) or Path(actual).resolve() != effective:
            raise ValueError("Actual FS:getUserPath differs; world untouched")
        if bng.control.queue_lua_command("return FS:fileExists(" + json.dumps(ART + "fixture.dae") + ")", response=True) != "true":
            raise ValueError("Fixture not mounted; restart intended profile manually")

        manifest = json.loads((ROOT / "generated/fixture.json").read_text(encoding="utf-8"))
        scene, car = scenario(manifest, "nzs_cap_" + uuid.uuid4().hex)
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
        time.sleep(5)
        settled = 0
        for _ in range(40):
            car.sensors.poll()
            if np.linalg.norm(dict(car.state)["vel"]) < 0.02:
                settled += 1
                if settled >= 5:
                    break
            else:
                settled = 0
            time.sleep(0.5)
        if settled < 5:
            raise RuntimeError("Vehicle did not settle below 0.02 m/s")

        for i in range(a.frames):
            offset = (i - (a.frames - 1) / 2.0) * a.stride
            progress("Frame %d/%d at camera x-offset %.3f" % (i + 1, a.frames, offset))
            for sensor in sensors:
                sensor.remove()
            sensors = []
            cam, lidar = make_sensors(bng, car, offset, a.timeout)
            sensors = [cam, lidar]
            time.sleep(1)

            bng.control.pause()
            bng.control.step(1)
            state_before = dict(car.state)
            poses_before = {"camera": sensor_pose(cam), "lidar": sensor_pose(lidar)}
            stamp_ns = int(round(time.monotonic() * 1e9))

            image_reading, _, _ = adhoc(cam, a.timeout)
            image = image_reading.get("colour")
            if image is None or image.size != (WIDTH, HEIGHT):
                raise RuntimeError("Missing/incorrect RGB image at frame %d" % i)
            frame_dir = frames_dir / ("frame_%03d" % i)
            frame_dir.mkdir(parents=True, exist_ok=True)
            image_path = frame_dir / "image.png"
            image.save(image_path)
            image_sha = hashlib.sha256(image_path.read_bytes()).hexdigest()

            cloud_reading, _, _ = adhoc(lidar, a.timeout)
            cloud = np.asarray(cloud_reading.get("pointCloud"), dtype=np.float64)
            if cloud.ndim != 2 or cloud.shape[1] != 3 or not np.isfinite(cloud).all():
                raise RuntimeError("Malformed LiDAR cloud at frame %d" % i)
            np.save(frame_dir / "cloud.npy", cloud, allow_pickle=False)

            state_after = dict(car.state)
            poses_after = {"camera": sensor_pose(cam), "lidar": sensor_pose(lidar)}
            stable, stability = pose_stability(state_before, state_after, poses_before, poses_after)

            cam_pos, cam_dir = poses_before["camera"]
            rotation, translation = world_to_camera(cam_pos, cam_dir)
            record = {
                "schema": "capture-frame/1", "epoch": epoch, "frame_index": i,
                "synchronized": bool(stable), "time_source": TIME_SOURCE,
                "image": {"file": "image.png", "sha256": image_sha, "width": WIDTH, "height": HEIGHT},
                "cloud": {"file": "cloud.npy", "coordinates": "world", "units": "m",
                          "points": int(len(cloud))},
                "camera": {"position": cam_pos.tolist(), "direction": cam_dir.tolist(),
                           "up": list(UP_WORLD)},
                "pose": {"source_frame": "smallgrid_world", "target_frame": "static_optical",
                         "R": rotation.tolist(), "translation_m": translation.tolist(), "units": "m"},
                "timing": {"stamp_ns": stamp_ns, "source": TIME_SOURCE, "epoch": epoch,
                           "time_kind": "acquisition"},
                "pose_stability": stability,
                "notes": ["paused then single-stepped before sampling; shared stamp assigned to image/cloud/pose"],
            }
            write_json(frame_dir / "capture.json", record)
            report["frames"].append({"frame_index": i, "stable": stable, "path": str(frame_dir),
                                     "camera_offset": offset, "points": int(len(cloud))})
            progress("Frame %d stable=%s points=%d" % (i, stable, len(cloud)))
            bng.control.resume()

        report["passed"] = bool(report["frames"]) and all(f["stable"] for f in report["frames"])
        return 0 if report["passed"] else 2
    finally:
        for sensor in reversed(sensors):
            try:
                sensor.remove()
            except Exception as exc:
                report.setdefault("cleanup_errors", []).append(str(exc))
        write_json(output / "capture_report.json", report)
        progress("Report: " + str(output / "capture_report.json"))
        bng.disconnect()


if __name__ == "__main__":
    raise SystemExit(capture_frames(parse_args()))
