"""Bounded native-headless American Road test; no viewer or semantic success claims."""
import json
import sys
import time
import uuid
from pathlib import Path

import numpy as np
from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.connection import Connection
from beamngpy.sensors import Camera, Electrics, Lidar

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "integration"))
from capture_american import read_spawn


def main():
    root = Path(__file__).resolve().parents[1]
    out = root / "wildos_lite" / "headless_tests" / uuid.uuid4().hex[:8]
    out.mkdir(parents=True, exist_ok=False)
    report = {"status": "started", "headless_requested": True,
              "scope": "motion and sensors only; not semantic navigation", "samples": []}
    bng = BeamNGpy("127.0.0.1", 64321, home=r"D:\BEAMNG\BeamNG.tech.v0.39.4.0",
                   user=r"D:\BEAMNG\rural-user", binary="Bin64/BeamNG.tech.x64.exe",
                   headless=True, nogpu=False, quit_on_close=True, socket_timeout=60)
    car, sensors = None, []

    def save():
        (out / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")

    def sample(stage):
        car.sensors.poll()
        state = dict(car.state)
        row = {"stage": stage, "sim_time": state.get("time"), "pos": list(state["pos"]),
               "speed": float(np.linalg.norm(state["vel"])),
               "electrics": {key: electrics.get(key) for key in
                             ("gear", "gear_index", "throttle", "brake", "parkingbrake", "rpm", "wheelspeed")}}
        report["samples"].append(row)
        print(json.dumps(row), flush=True)
        save()
        return row

    try:
        print("REPORT " + str(out / "report.json"), flush=True)
        spawn = read_spawn(Path(r"D:\BEAMNG\rural-user\current\mods\american_road_0.39.zip"), "spawns_city")
        bng.open(launch=True, listen_ip="127.0.0.1")
        report["launch_command"] = bng.last_command_line
        if not bng.last_command_line or "-headless" not in bng.last_command_line.split():
            raise RuntimeError("Native -headless launch was not verified; refusing test")
        scene = Scenario("american_road", "headless_" + out.name)
        car = Vehicle("headless_ego", model="pickup", part_config="vehicles/pickup/d15_4wd_A.pc")
        electrics = Electrics()
        car.sensors.attach("electrics", electrics)
        scene.add_vehicle(car, pos=tuple(spawn["position"]), rot_quat=tuple(spawn["rot_quat"]), safe_spawn=True)
        scene.make(bng)
        print("Loading American Road", flush=True)
        bng.scenario.load(scene, connect_player_vehicle=False, connect_existing_vehicles=False)
        endpoint = bng.vehicles.start_connection(car, car.extensions)
        car.connection = Connection("127.0.0.1", int(endpoint["result"]), timeout=60)
        car.connect(bng)
        bng.scenario.start()
        car.ai.set_mode("disabled")
        car.set_shift_mode("realistic_automatic")
        bng.control.pause()
        for index in range(30):
            car.control(throttle=0, brake=1, parkingbrake=1)
            bng.control.step(10)
            parked = sample("settle_%02d" % index)
            if index >= 4 and parked["speed"] < 0.1:
                break
        else:
            raise RuntimeError("Vehicle did not remain stopped; motion test refused")
        before = sample("before")
        # Select the drive gear once, not on every control tick.
        car.control(throttle=0.2, brake=0, parkingbrake=0, steering=0, gear=1)
        for index in range(12):
            bng.control.step(10)
            row = sample("drive_%02d" % index)
            if row["electrics"]["gear"] in ("P", "R", "N"):
                report["motion_blocker"] = "Requested gear=1, observed " + str(row["electrics"]["gear"])
                break
            if row["speed"] > 3 or np.linalg.norm(np.asarray(row["pos"]) - before["pos"]) > 5:
                break
        car.control(throttle=0, brake=1, parkingbrake=1)
        bng.control.step(120)
        after = sample("after")
        report["displacement_m"] = float(np.linalg.norm(np.asarray(after["pos"]) - before["pos"]))
        camera = Camera("headless_cam", bng, car, pos=(0,-1.5,2.35), dir=(0,-1,0), up=(0,0,1),
                        resolution=(640,360), field_of_view_y=60, requested_update_time=0.2,
                        is_using_shared_memory=False, is_render_colours=True,
                        is_render_depth=False, is_render_annotations=False, is_visualised=False)
        sensors.append(camera)
        lidar = Lidar("headless_lidar", bng, car, pos=(0,0,2.8), dir=(0,-1,0), up=(0,0,1),
                      requested_update_time=0.2, vertical_resolution=32, vertical_angle=90,
                      max_distance=40, density=100, is_360_mode=True,
                      is_using_shared_memory=False, is_visualised=False)
        sensors.append(lidar)
        bng.control.resume()
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            car.control(throttle=0, brake=1, parkingbrake=1)
            image = camera.poll().get("colour")
            cloud = np.asarray(lidar.poll().get("pointCloud"))
            if image is not None and cloud.ndim == 2 and len(cloud):
                image.convert("RGB").save(out / "camera.png")
                np.save(out / "cloud_raw.npy", cloud, allow_pickle=False)
                report["camera_size"] = list(image.size)
                report["camera_std"] = float(np.asarray(image).std())
                report["lidar_points"] = len(cloud)
                report["lidar_finite_fraction"] = float(np.isfinite(cloud).all(axis=1).mean())
                break
            time.sleep(0.2)
        report["sensor_passed"] = bool(report.get("camera_std", 0) > 1 and
                                        report.get("lidar_finite_fraction", 0) > 0.99)
        report["motion_passed"] = bool(not report.get("motion_blocker") and before["speed"] < 0.1 and
                                       report["displacement_m"] > 0.5)
        report["status"] = "passed" if report["sensor_passed"] and report["motion_passed"] else "incomplete"
    except Exception as exc:
        report.update(status="failed", error=type(exc).__name__ + ": " + str(exc))
        print(report["error"], flush=True)
    finally:
        if car is not None:
            try:
                car.control(throttle=0, brake=1, parkingbrake=1)
            except Exception:
                pass
        for sensor in reversed(sensors):
            try:
                sensor.remove()
            except Exception:
                pass
        save()
        bng.close()
        print("STATUS " + report["status"], flush=True)
    return 0 if report["status"] == "passed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
