"""Continuous live stream of the mounted rig (no freeze), on display :6.

Unlike live_capture (pause/step snapshots), this keeps the simulation running,
holds the ego still (neutral gear + parking brake re-sent every cycle) and polls
the mounted camera/LiDAR continuously, refreshing an OpenCV window:

    [ camera ]            [ camera + LiDAR depth points ]

ExploRFM traversability/frontier are recomputed asynchronously (subprocess) on a
throttled cadence and overlaid when ready. The OpenCV window is raised to the
front so BeamNG's fullscreen does not hide it.

    python -B integration/live_stream.py --home ... --user ... --replace-current \
        --seconds 120 --checkpoint-root ~/ckpts --source-root .../nebula2-wildos
"""
import argparse
import subprocess
import sys
import threading
import time
import uuid
from pathlib import Path

import cv2
import numpy as np

import capture_frames as cf
import live_capture as lc

SENSOR_CAL_ROOT = Path(__file__).resolve().parent.parent / "sensor_calibration"
W, H = cf.WIDTH, cf.HEIGHT


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--home", type=Path, required=True)
    p.add_argument("--user", type=Path, required=True)
    p.add_argument("--replace-current", action="store_true")
    p.add_argument("--port", type=int, default=64256)
    p.add_argument("--timeout", type=float, default=60)
    p.add_argument("--gfx", choices=("dx11", "vk"))
    p.add_argument("--seconds", type=float, default=120.0)
    p.add_argument("--lidar-period", type=float, default=0.5)
    p.add_argument("--checkpoint-root", type=Path)
    p.add_argument("--source-root", type=Path)
    p.add_argument("--heads-python", default="/home/ly/vision-cu128/bin/python")
    p.add_argument("--heads-every", type=float, default=6.0)
    p.add_argument("--output", type=Path)
    a = p.parse_args(argv)
    if not a.replace_current:
        p.error("--replace-current is required")
    return a


class HeadsWorker(threading.Thread):
    def __init__(self, py, script, ckpt, src, root, period):
        super().__init__(daemon=True)
        self.py, self.script, self.ckpt, self.src = py, Path(script), ckpt, src
        self.root, self.period = Path(root), period
        self.latest = None
        self.maps = None
        self.lock = threading.Lock()
        self.stop = threading.Event()

    def offer(self, frame_rgb):
        with self.lock:
            self.latest = frame_rgb

    def get(self):
        with self.lock:
            return self.maps

    def run(self):
        counter = 0
        while not self.stop.is_set():
            self.stop.wait(self.period)
            with self.lock:
                image = self.latest
            if image is None:
                continue
            counter += 1
            out_dir = self.root / ("stream_%04d" % counter)
            img_path = out_dir.with_suffix(".png")
            out_dir.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(img_path), image)
            subprocess.run([self.py, "-B", str(self.script), "--image", str(img_path),
                            "--checkpoint-root", str(self.ckpt), "--source-root", str(self.src),
                            "--output", str(out_dir), "--heatmap", "relative", "--smooth"], check=False)
            trav = out_dir / "traversability.npy"
            front = out_dir / "frontier.npy"
            maps = (np.load(trav) if trav.is_file() else None, np.load(front) if front.is_file() else None)
            with self.lock:
                self.maps = maps


def main(argv=None):
    a = parse_args(argv)
    sys.path.insert(0, str(SENSOR_CAL_ROOT))
    import importlib.metadata
    from fixture import ROOT, ART, MOD, scenario
    from install import user_paths, verified_mod
    from inspect_local import inspect_home

    if importlib.metadata.version("beamngpy") != "1.36":
        raise ValueError("Requires beamngpy==1.36")
    inspect_home(a.home)
    parent, effective = user_paths(a.user)

    from beamngpy import BeamNGpy
    from beamngpy.connection import Connection

    bng = BeamNGpy("127.0.0.1", a.port, home=str(a.home.resolve()), user=str(parent),
                   binary="BinLinux/BeamNG.tech.x64", quit_on_close=False,
                   socket_timeout=a.timeout, gfx=a.gfx)
    cf.progress("Connecting to BeamNG")
    bng.open(launch=True, listen_ip="127.0.0.1")
    import json
    manifest = json.loads((ROOT / "generated/fixture.json").read_text(encoding="utf-8"))
    scene, car = scenario(manifest, "nzs_stream_" + uuid.uuid4().hex)
    scene.make(bng)
    bng.scenario.load(scene, connect_player_vehicle=False, connect_existing_vehicles=False)
    endpoint = bng.vehicles.start_connection(car, car.extensions)
    car.connection = Connection("127.0.0.1", int(endpoint["result"]), timeout=a.timeout)
    car.connect(bng)
    bng.scenario.start()
    car.ai.set_mode("disabled")
    car.set_shift_mode("realistic_automatic")

    def hold():
        car.control(throttle=0, brake=1, parkingbrake=1, steering=0, gear=0)

    hold()
    bng.control.resume()
    time.sleep(2.0)
    cf.progress("Attaching mounted rig")
    rig = lc.make_rig(bng, car)
    time.sleep(1.0)

    heads = None
    if a.checkpoint_root and a.source_root:
        root = Path(__file__).resolve().parent.parent / "vocabulary_query" / "live_heads"
        root.mkdir(parents=True, exist_ok=True)
        script = Path(__file__).resolve().parent.parent / "vocabulary_query" / "explorfm_heads.py"
        heads = HeadsWorker(a.heads_python, script, a.checkpoint_root, a.source_root, root, a.heads_every)
        heads.start()

    cv2.namedWindow("wildos-stream", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("wildos-stream", int(2 * W * 0.62), int(H * 0.62))
    started, last_lidar, last_cloud = time.time(), 0.0, np.zeros((0, 3))
    frames = 0
    try:
        while time.time() - started < a.seconds:
            hold()
            reading, _, _ = cf.adhoc(rig["camera"], a.timeout)
            colour = reading.get("colour")
            if colour is None:
                cv2.waitKey(20)
                continue
            rgb = np.asarray(colour)
            frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR) if rgb.ndim == 3 else rgb
            if time.time() - last_lidar > a.lidar_period:
                cloud_reading, _, _ = cf.adhoc(rig["lidar"], a.timeout)
                cloud = np.asarray(cloud_reading.get("pointCloud"), dtype=np.float64)
                position = np.asarray(rig["lidar"].get_position(), float)
                if cloud.ndim == 2 and cloud.shape[1] == 3:
                    last_cloud = lc.cloud_to_world(cloud, position)[0]
                last_lidar = time.time()
            cam_pos = np.asarray(rig["camera"].get_position(), float)
            cam_dir = np.asarray(rig["camera"].get_direction(), float)
            rotation, translation = cf.world_to_camera(cam_pos, cam_dir)
            panel = np.zeros((H, 2 * W, 3), np.uint8)
            panel[:, 0:W] = frame
            panel[:, W:2 * W] = lv_draw(frame, last_cloud, rotation, translation)
            maps = heads.get() if heads else None
            if maps and maps[0] is not None:
                panel[:, 0:W] = lv_heat(frame, maps[0])
                panel[:, W:2 * W] = lv_heat(panel[:, W:2 * W], maps[1]) if maps[1] is not None else panel[:, W:2 * W]
            cv2.putText(panel, "camera", (12, 34), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(panel, "camera+LiDAR" + ("  trav/frontier" if maps else "  heads..."),
                        (W + 12, 34), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
            view = cv2.resize(panel, None, fx=0.62, fy=0.62, interpolation=cv2.INTER_AREA)
            cv2.imshow("wildos-stream", view)
            cv2.setWindowProperty("wildos-stream", cv2.WND_PROP_TOPMOST, 1)
            if heads:
                heads.offer(frame)
            key = cv2.waitKey(20)
            frames += 1
            if key in (ord("q"), 27):
                break
    finally:
        if heads:
            heads.stop.set()
        car.control(throttle=0, brake=1, parkingbrake=1, steering=0, gear=0)
        for sensor in reversed(list(rig.values())):
            try:
                sensor.remove()
            except Exception:
                pass
        cv2.destroyAllWindows()
        bng.disconnect()
    cf.progress("stream done, frames=%d" % frames)
    return 0


def lv_draw(frame, cloud, rotation, translation):
    vis = frame.copy()
    if cloud.shape[0] == 0:
        return vis
    camera = cloud @ np.asarray(rotation, float).T + np.asarray(translation, float)
    depth = camera[:, 2]
    keep = depth > 0.1
    camera, depth = camera[keep], depth[keep]
    fx, fy, cx, cy = 554.2562584220408, 554.2562584220408, W / 2.0, H / 2.0
    u = fx * camera[:, 0] / depth + cx
    v = fy * camera[:, 1] / depth + cy
    inside = (u >= 0) & (u < W) & (v >= 0) & (v < H)
    u, v, depth = u[inside].astype(int), v[inside].astype(int), depth[inside]
    if len(u) == 0:
        return vis
    if len(u) > 25000:
        stride = max(1, len(u) // 25000)
        u, v, depth = u[::stride], v[::stride], depth[::stride]
    norm = np.clip((depth - depth.min()) / max(depth.max() - depth.min(), 1e-6), 0, 1)
    colors = (np.array([255, 0, 0]) * (1 - norm)[:, None] + np.array([0, 0, 255]) * norm[:, None])
    vis[v, u] = colors.astype(np.uint8)
    return vis


def lv_heat(frame, heat):
    heat = np.clip(np.asarray(heat, np.float32), 0.0, 1.0)
    heat = cv2.resize(heat, (W, H), interpolation=cv2.INTER_LINEAR)
    colored = cv2.applyColorMap((heat * 255).astype(np.uint8), cv2.COLORMAP_JET)
    return cv2.addWeighted(frame, 0.55, colored, 0.45, 0)


if __name__ == "__main__":
    raise SystemExit(main())
