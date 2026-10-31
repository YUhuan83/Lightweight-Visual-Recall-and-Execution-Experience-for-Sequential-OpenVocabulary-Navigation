"""Local BeamNG 3D live pathfinding demo (no server, no ROS).

Launches local BeamNG.tech, spawns the pickup on a flat map with procedural
obstacle blocks, mounts a 360 LiDAR, and repeatedly: LiDAR -> occupancy grid ->
A* -> BeamNG AI follows the polyline while the path is drawn in the 3D world with
debug.add_polyline. Re-plans each cycle until the goal is reached.

    .beamng-venv\\Scripts\\python.exe tools\\beamng_pathfind_local.py --smoke
    .beamng-venv\\Scripts\\python.exe tools\\beamng_pathfind_local.py
"""
import argparse
import sys
import time
import uuid
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from wildos_lite import graph, planner  # noqa: E402

DEFAULT_HOME = Path(r"D:\BEAMNG\BeamNG.tech.v0.39.4.0")
DEFAULT_OBSTACLES = [
    ((0.0, -25.0, 0.75), (7.0, 3.0, 1.5)),
    ((4.0, -38.0, 0.75), (3.0, 7.0, 1.5)),
    ((-5.0, -52.0, 0.75), (6.0, 3.0, 1.5)),
]


def progress(msg):
    print(time.strftime("[%H:%M:%S] ") + msg, flush=True)


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--home", type=Path, default=DEFAULT_HOME)
    p.add_argument("--port", type=int, default=64256)
    p.add_argument("--map", default="smallgrid")
    p.add_argument("--spawn", default="0,4,0.65")
    p.add_argument("--goal", default="0,-70")
    p.add_argument("--speed", type=float, default=7.0)
    p.add_argument("--cycle-seconds", type=float, default=4.0)
    p.add_argument("--max-cycles", type=int, default=25)
    p.add_argument("--goal-radius", type=float, default=4.0)
    p.add_argument("--timeout", type=float, default=120)
    p.add_argument("--smoke", action="store_true", help="Launch, capture one LiDAR cloud, exit")
    return p.parse_args(argv)


def adhoc(sensor, timeout):
    deadline = time.monotonic() + timeout
    start = time.monotonic()
    request = sensor.send_ad_hoc_poll_request()
    while not sensor.is_ad_hoc_poll_request_ready(request):
        if time.monotonic() - start > timeout or time.monotonic() > deadline:
            raise TimeoutError("LiDAR poll timed out")
        time.sleep(0.05)
    return sensor.collect_ad_hoc_poll_request(request)


def point_cloud_to_world(cloud, lidar_position):
    ground = float(np.percentile(cloud[:, 2], 5))
    if -5.0 < ground < 5.0:
        return cloud, "raw_world"
    return cloud + np.asarray(lidar_position, float), "translated_local"


def main(argv=None):
    a = parse_args(argv)
    spawn = [float(v) for v in a.spawn.split(",")]
    goal = [float(v) for v in a.goal.split(",")]

    from beamngpy import BeamNGpy, Scenario, Vehicle, ProceduralCube
    from beamngpy.connection import Connection
    from beamngpy.sensors import Lidar

    bng = BeamNGpy("127.0.0.1", a.port, home=str(a.home), binary="Bin64/BeamNG.tech.x64.exe",
                   quit_on_close=False, socket_timeout=a.timeout)
    car, lidar, line_id = None, None, None
    try:
        progress("launching local BeamNG: %s" % a.home)
        bng.open(launch=True, listen_ip="127.0.0.1")
        scene = Scenario(a.map, "pf_" + uuid.uuid4().hex, description="local 3D pathfinding demo")
        car = Vehicle("pf_ego", model="pickup", part_config="vehicles/pickup/d15_4wd_A.pc")
        scene.add_vehicle(car, pos=tuple(spawn), rot_quat=(0, 0, 0, 1))
        for i, (pos, size) in enumerate(DEFAULT_OBSTACLES):
            scene.add_procedural_mesh(ProceduralCube(pos, size, "pf_block_%d" % i))
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

        progress("mounting 360 LiDAR")
        lidar = Lidar("pf_lidar", bng, car, pos=(0, 0, 3.6), dir=(0, -1, -0.45), up=(0, 0, 1),
                      is_static=False, is_dir_world_space=False, requested_update_time=-1,
                      vertical_resolution=64, vertical_angle=90, horizontal_angle=360,
                      max_distance=60, density=20, is_360_mode=True,
                      is_using_shared_memory=False, is_visualised=False, is_annotated=False)
        time.sleep(1.5)

        if a.smoke:
            bng.control.pause()
            bng.control.step(1)
            car.sensors.poll()
            reading = adhoc(lidar, a.timeout)
            raw = reading.get("pointCloud")
            if raw is None:
                progress("smoke: no pointCloud, reading keys=%s" % list(reading.keys()))
                return 1
            cloud = np.asarray(raw, float)
            pos = np.asarray(dict(car.state)["pos"], float)
            world, frame = point_cloud_to_world(cloud, np.asarray(lidar.get_position(), float))
            progress("smoke: points=%d frame=%s car_xy=(%.1f,%.1f) z=[%.2f,%.2f]" % (
                len(cloud), frame, pos[0], pos[1], world[:, 2].min(), world[:, 2].max()))
            return 0

        for cycle in range(a.max_cycles):
            bng.control.pause()
            bng.control.step(1)
            car.sensors.poll()
            state = dict(car.state)
            pos = np.asarray(state["pos"], float)
            reading = adhoc(lidar, a.timeout)
            raw = reading.get("pointCloud")
            if raw is None:
                progress("cycle %d: no pointCloud (keys=%s)" % (cycle, list(reading.keys())))
                break
            cloud = np.asarray(raw, float)
            world, frame = point_cloud_to_world(cloud, np.asarray(lidar.get_position(), float))
            ground = float(np.percentile(world[:, 2], 5))

            height = world[:, 2] - ground
            obstacles = world[(height > 0.4) & (height < 3.0)][:, :2]
            obstacles = obstacles[np.linalg.norm(obstacles - pos[:2], axis=1) > 3.0]

            grid = planner.build_grid(obstacles, (pos[0], pos[1]), (goal[0], goal[1]),
                                      half_extent=35.0, resolution=0.5, inflation=1.8)
            path = planner.simplify(planner.astar(grid, (pos[0], pos[1]), (goal[0], goal[1])),
                                    obstacles, obstacle_radius=1.8)
            distance = float(np.linalg.norm(pos[:2] - np.asarray(goal, float)))
            progress("cycle %d: pos=(%.1f,%.1f) cloud_frame=%s obstacles=%d path=%d dist_goal=%.1f" % (
                cycle, pos[0], pos[1], frame, len(obstacles), len(path), distance))
            if distance <= a.goal_radius:
                progress("goal reached")
                break
            if not path:
                progress("no path; stopping")
                break

            line = [{"pos": (float(x), float(y), ground + 0.6), "speed": a.speed} for x, y in path]
            car.ai.set_mode("traffic")
            car.ai.set_line(line, cling=True)
            if line_id is not None:
                try:
                    bng.debug.remove_polyline(line_id)
                except Exception:
                    pass
            line_id = bng.debug.add_polyline([(float(x), float(y), ground + 0.4) for x, y in path],
                                             (0.1, 1.0, 0.1, 1.0), cling=True)
            bng.control.resume()
            time.sleep(a.cycle_seconds)

        progress("done")
        return 0
    finally:
        try:
            if car is not None:
                car.control(throttle=0, brake=1, parkingbrake=1, steering=0, gear=0)
        except Exception:
            pass
        bng.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
