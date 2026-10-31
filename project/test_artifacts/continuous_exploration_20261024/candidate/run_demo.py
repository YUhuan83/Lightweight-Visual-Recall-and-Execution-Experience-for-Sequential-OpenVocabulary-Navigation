"""WildOS-lite American Road demo (headless): text goal -> nodes -> plan -> drive.

Each cycle: freeze, capture camera+LiDAR, run vocabulary_query (subprocess) on the
image, localize the target (LiDAR-in-mask median depth), build/score the candidate
node graph, select a goal, and apply the pure-pursuit control for a short drive
window. Stops when the target is inside the success radius.

    python -B run_demo.py --home ... --user ... --map-zip ... --spawn spawns_road1 \
        --query "stop sign" --source-root ... --checkpoint-root ~/ckpts \
        --output ~/wildos-lite-out --replace-current
"""
import argparse
import json
import subprocess
import sys
import time
import uuid
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "integration"))

import capture_american as ca
import capture_frames as cf
from object_search import triangulation as T
from wildos_lite.agent import Config, Observation, Policy

K = np.array([[554.2562584220408, 0.0, 480.0], [0.0, 554.2562584220408, 320.0], [0.0, 0.0, 1.0]])


def load_mask(path):
    import cv2
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    return None if image is None else (image > 127)


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--home", type=Path, required=True)
    p.add_argument("--user", type=Path, required=True)
    p.add_argument("--map-zip", type=Path, required=True)
    p.add_argument("--spawn", default="spawns_road1")
    p.add_argument("--query", required=True)
    p.add_argument("--source-root", type=Path, required=True)
    p.add_argument("--checkpoint-root", type=Path, required=True)
    p.add_argument("--query-python", default="/home/ly/vision-cu128/bin/python")
    p.add_argument("--query-script", type=Path, default=str(ROOT / "vocabulary_query" / "query.py"))
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--cycles", type=int, default=30)
    p.add_argument("--drive-seconds", type=float, default=1.5)
    p.add_argument("--steer-sign", type=float, default=1.0)
    p.add_argument("--replace-current", action="store_true")
    p.add_argument("--port", type=int, default=64256)
    p.add_argument("--gfx", choices=("dx11", "vk"), default="vk")
    p.add_argument("--timeout", type=float, default=600)
    a = p.parse_args(argv)
    if not a.replace_current:
        p.error("--replace-current is required")
    return a


def main(argv=None):
    a = parse_args(argv)
    from beamngpy import BeamNGpy, Scenario, Vehicle
    from beamngpy.connection import Connection

    out = a.output.expanduser().resolve() / ("wildos-lite-" + uuid.uuid4().hex[:8])
    out.mkdir(parents=True, exist_ok=True)
    spawn = ca.read_spawn(a.map_zip.expanduser().resolve(), a.spawn)
    cf.progress("demo output: %s spawn=%s query='%s'" % (out, a.spawn, a.query))

    bng = BeamNGpy("127.0.0.1", a.port, home=str(a.home.resolve()), user=str(a.user.resolve()),
                   binary="BinLinux/BeamNG.tech.x64", quit_on_close=False,
                   socket_timeout=a.timeout, gfx=a.gfx)
    policy = Policy(Config(steer_sign=a.steer_sign))
    log, car, rig = [], None, {}
    try:
        bng.open(launch=True, listen_ip="127.0.0.1")
        scene = Scenario("american_road", "am_demo_" + uuid.uuid4().hex, description="WildOS-lite demo")
        car = Vehicle("am_ego", model="pickup", part_config=ca.PART_CONFIG)
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
        rig = ca.make_rig(bng, car)
        time.sleep(1.0)

        for cycle in range(a.cycles):
            bng.control.pause()
            bng.control.step(1)
            state = dict(car.state)
            pos = np.asarray(state["pos"], float)
            fwd = np.asarray(state.get("dir", [0.0, -1.0, 0.0]), float)[:2]
            if float(np.linalg.norm(fwd)) < 1e-6:
                fwd = np.array([0.0, -1.0])

            cam_reading, _, _ = cf.adhoc(rig["camera"], a.timeout)
            image = cam_reading.get("colour")
            cloud_reading, _, _ = cf.adhoc(rig["lidar"], a.timeout)
            raw = np.asarray(cloud_reading.get("pointCloud"), float)
            cam_pos = np.asarray(rig["camera"].get_position(), float)
            cam_dir = np.asarray(rig["camera"].get_direction(), float)
            lidar_pos = np.asarray(rig["lidar"].get_position(), float)
            world_cloud = ca.cloud_to_world(raw, lidar_pos)[0]

            frame_dir = out / ("cycle_%03d" % cycle)
            frame_dir.mkdir(exist_ok=True)
            image_path = frame_dir / "image.png"
            image.save(image_path)
            np.save(frame_dir / "cloud.npy", world_cloud, allow_pickle=False)

            qout = frame_dir / "query"
            qout.mkdir(exist_ok=True)
            subprocess.run([a.query_python, "-B", str(a.query_script),
                            "--source-root", str(a.source_root), "--checkpoint-root", str(a.checkpoint_root),
                            "--image", str(image_path), "--output", str(qout), "--query", a.query],
                           check=False)
            mask = load_mask(qout / "query_000_region_mask.png")
            rotation, _ = cf.world_to_camera(cam_pos, cam_dir)
            target_xy, seen = None, False
            mask_pixels = 0
            if mask is not None:
                mask_pixels = int(mask.sum())
                lock = T.localize_in_range(world_cloud, mask, K, rotation.T, cam_pos, min_points=150)
                if lock is not None:
                    target_xy, seen = [float(lock[0]), float(lock[1])], True

            decision = policy.decide(Observation(
                position_xy=[float(pos[0]), float(pos[1])],
                forward_xy=[float(fwd[0]), float(fwd[1])],
                lidar_xyz=world_cloud, target_xy=target_xy, target_seen=seen))
            entry = {"cycle": cycle, "car_xy": [float(pos[0]), float(pos[1])], "phase": decision.phase,
                     "goal_xy": [float(v) for v in decision.goal_xy], "target_xy": target_xy,
                     "command": decision.command, "mask_pixels": mask_pixels,
                     "target_distance_m": decision.target_distance_m}
            log.append(entry)
            cf.progress(json.dumps(entry))
            if decision.phase == "FOUND":
                break

            command = decision.command
            car.control(steering=float(command["steering"]), throttle=float(command["throttle"]),
                        brake=float(command["brake"]), parkingbrake=0, gear=1)
            bng.control.resume()
            time.sleep(a.drive_seconds)

        report = {"schema": "wildos-lite-demo/1", "query": a.query, "spawn": a.spawn,
                  "cycles": len(log), "final_phase": log[-1]["phase"] if log else None,
                  "final_target_xy": log[-1]["target_xy"] if log else None, "log": log}
        (out / "demo_report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        cf.progress(json.dumps({"cycles": len(log), "phase": report["final_phase"],
                                "target": report["final_target_xy"]}))
        return 0
    finally:
        for sensor in reversed(list(rig.values())):
            try:
                sensor.remove()
            except Exception:
                pass
        try:
            if car is not None:
                car.control(throttle=0, brake=1, parkingbrake=1, steering=0, gear=0)
        except Exception:
            pass
        try:
            bng.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
