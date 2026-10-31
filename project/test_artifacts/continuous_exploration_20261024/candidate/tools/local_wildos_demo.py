"""Local (Windows) full WildOS closed loop on BeamNG American Road, third-person.

Single process (system python with CUDA torch, run with -s) drives BeamNG via
BeamNGpy and talks to a PERSISTENT vision worker (SigLIP2 + C-RADIO + trav/frontier
heads loaded once) over JSON lines.

Semantic input: --query or type a new target + Enter at any time.
Loop: camera + LiDAR -> text query + heads (one worker call) -> target localization
(LiDAR-in-mask) -> A* on the LiDAR occupancy grid -> BeamNG AI follows the polyline
(third-person chase camera) -> re-plan; when the target is not visible, explore
toward the best drivable frontier using the WildOS additive frontier/heading/
reachability scoring (explore.py). Stops inside the success radius.

The demo keeps only the BeamNG view: RGB/LiDAR feed the controller but are never
shown as extra windows, and per-cycle artifacts are deleted unless --save-artifacts.
Low-bandwidth runs use --headless and/or a larger --vision-period.

    python -s tools\\local_wildos_demo.py --query "house"
"""
import argparse
import json
import math
import os
import platform
import shutil
import subprocess
import sys
import threading
import time
import uuid
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "integration"))

import capture_american as ca  # noqa: E402
import capture_frames as cf  # noqa: E402
from object_search import triangulation as T  # noqa: E402
from wildos_lite import controller, explore, graph, planner  # noqa: E402

FOV_Y_DEG = 60.0


def camera_intrinsics(size, fov_y_deg=FOV_Y_DEG):
    """Pinhole K and horizontal FOV for a WxH image with vertical FOV fov_y_deg."""
    width, height = int(size[0]), int(size[1])
    fy = (height / 2.0) / math.tan(math.radians(fov_y_deg) / 2.0)
    fx = fy
    k = np.array([[fx, 0.0, width / 2.0], [0.0, fy, height / 2.0], [0.0, 0.0, 1.0]])
    h_fov = math.degrees(2.0 * math.atan(math.tan(math.radians(fov_y_deg) / 2.0) * width / height))
    return k, h_fov


IS_WINDOWS = platform.system() == "Windows"

if IS_WINDOWS:
    DEFAULT_HOME = Path(r"D:\BEAMNG\BeamNG.tech.v0.39.4.0")
    DEFAULT_USER = Path(r"D:\BEAMNG\rural-user")
    DEFAULT_ZIP = Path(r"D:\BEAMNG\rural-user\current\mods\american_road_0.39.zip")
    DEFAULT_CKPT = Path(r"C:\Users\Yu\ckpts")
    DEFAULT_SOURCE = Path(r"C:\Users\Yu\nebula2-wildos")
    DEFAULT_OUTPUT = Path(__file__).resolve().parent.parent / "wildos_lite" / "local_out"
    DEFAULT_BINARY = "Bin64/BeamNG.tech.x64.exe"
    DEFAULT_GFX = None
    DEFAULT_WORKER_PYTHON = sys.executable
else:
    DEFAULT_HOME = Path("/home/ly/beamng-install-0.39.4.0/BeamNG.tech.v0.39.4.0")
    DEFAULT_USER = Path("/home/ly/beamng-american-user")
    DEFAULT_ZIP = Path("/home/ly/beamng-maps/american_road_0.39.zip")
    DEFAULT_CKPT = Path("/home/ly/ckpts")
    DEFAULT_SOURCE = Path("/home/ly/wildos-sim/wildos_port/repro_check/src/nebula2-wildos")
    DEFAULT_OUTPUT = Path("/home/ly/wildos-lite-out")
    DEFAULT_BINARY = "BinLinux/BeamNG.tech.x64"
    DEFAULT_GFX = "vk"
    DEFAULT_WORKER_PYTHON = "/home/ly/vision-cu128/bin/python"

STATE = {"target": None, "quit": False, "last": time.monotonic()}
WATCHDOG_SECONDS = 300.0


def progress(msg):
    STATE["last"] = time.monotonic()
    print(time.strftime("[%H:%M:%S] ") + msg, flush=True)


def watchdog():
    """Abort loudly if no progress line for WATCHDOG_SECONDS, so a blocked
    BeamNG socket can never freeze the terminal forever."""
    while not STATE["quit"]:
        time.sleep(5.0)
        idle = time.monotonic() - STATE["last"]
        if idle > WATCHDOG_SECONDS:
            print("WATCHDOG: no progress for %.0fs; aborting" % idle, flush=True)
            os._exit(3)


def load_mask(path):
    import cv2
    image = cv2.imdecode(np.fromfile(str(path), dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
    return None if image is None else (image > 127)


def image_dimensions(image):
    """Return (width, height) for PIL or NumPy Camera colour output."""
    if image is None:
        return None
    size = getattr(image, "size", None)
    if isinstance(size, tuple) and len(size) == 2:
        return int(size[0]), int(size[1])
    array = np.asarray(image)
    if array.ndim >= 2:
        return int(array.shape[1]), int(array.shape[0])
    return None


def image_size_matches(image_size, width, height):
    """Compare image dimensions using scalar conversion, never array truthiness."""
    if image_size is None:
        return False
    try:
        values = np.asarray(image_size).reshape(-1)
        return len(values) == 2 and int(values[0]) == int(width) and int(values[1]) == int(height)
    except (TypeError, ValueError, OverflowError):
        return False


def bearing_deg(vector):
    """Absolute world bearing (degrees, [-180,180]) of a 2D direction vector."""
    v = np.asarray(vector, float).reshape(2)
    return float(math.degrees(math.atan2(v[1], v[0])))


def save_camera_image(image, path):
    """Save PIL or NumPy Camera output as RGB PNG."""
    if hasattr(image, "save"):
        image.save(path)
        return
    from PIL import Image
    array = np.asarray(image)
    if array.ndim != 3 or array.shape[2] not in (3, 4):
        raise RuntimeError("invalid Camera colour array shape: %s" % (array.shape,))
    Image.fromarray(array[:, :, :3].astype(np.uint8), mode="RGB").save(path)


def select_region_mask(path, response):
    """Keep only the largest kept region, so multi-instance queries (e.g. red
    house) localize against one object instead of the union of all of them."""
    mask = load_mask(path)
    if mask is None:
        return None
    regions = (response or {}).get("regions") or []
    if not regions:
        return mask
    best = max(regions, key=lambda r: r.get("area_pixels", 0))
    x1, y1, x2, y2 = (int(v) for v in best["box_xyxy"])
    h, w = mask.shape
    x1, y1 = max(x1, 0), max(y1, 0)
    x2, y2 = min(x2, w), min(y2, h)
    out = np.zeros_like(mask)
    out[y1:y2, x1:x2] = mask[y1:y2, x1:x2]
    return out


def finite_head_map(value, name):
    """Accept only a finite non-empty 2D head output."""
    if value is None:
        return None
    array = np.asarray(value, dtype=float)
    if array.ndim != 2 or not array.size or not np.isfinite(array).all():
        progress("  invalid %s map; ignoring it" % name)
        return None
    return array


def target_track_update(track, candidate, max_jump, confirmations, miss_tolerance=2, window=6):
    """Confirm a spatially consistent target within a recent observation window."""
    if candidate is None:
        track["misses"] = track.get("misses", 0) + 1
        if track["misses"] <= miss_tolerance:
            return track.get("confirmed")
        track["candidate"] = None
        track["count"] = 0
        track["confirmed"] = None
        track["observations"] = []
        return None
    candidate = np.asarray(candidate, dtype=float)[:2]
    track["misses"] = 0
    confirmed = track.get("confirmed")
    if confirmed is not None:
        # Once confirmed, keep the world target fixed for this run. Small
        # per-frame depth changes otherwise accumulate into a moving goal.
        return track["confirmed"]
    observations = [np.asarray(v, dtype=float)[:2] for v in track.get("observations", [])]
    if observations:
        center = np.median(np.asarray(observations), axis=0)
        if float(np.linalg.norm(candidate - center)) > max_jump:
            observations = []
    observations.append(candidate)
    track["observations"] = observations[-int(window):]
    center = np.median(np.asarray(track["observations"]), axis=0)
    track["candidate"] = center
    track["count"] = len(track["observations"])
    if track["count"] >= confirmations:
        track["confirmed"] = tuple(float(v) for v in center)
    return track["confirmed"]


def short_endpoint(path, chunk_metres):
    """Return a point no farther than chunk_metres along a planned path."""
    if len(path) < 2:
        return np.asarray(path[0], float) if path else None
    current = np.asarray(path[0], dtype=float)
    endpoint = current.copy()
    travelled = 0.0
    for point in path[1:]:
        point = np.asarray(point, float)
        delta = point - current
        segment = float(np.linalg.norm(delta))
        if segment <= 1e-3:
            continue
        remaining = max(0.0, float(chunk_metres) - travelled)
        if segment >= remaining:
            endpoint = current + delta * (remaining / segment)
            return endpoint[:2]
        endpoint = point
        travelled += segment
        current = point
    return endpoint[:2]


def direct_target_path(position_xy, target_xy, obstacles, chunk_metres, clearance=1.8):
    """Return a short direct target segment only when LiDAR sees it as clear."""
    start = np.asarray(position_xy, dtype=float)[:2]
    target = np.asarray(target_xy, dtype=float)[:2]
    delta = target - start
    distance = float(np.linalg.norm(delta))
    if distance <= 1e-6:
        return [tuple(start), tuple(start)]
    endpoint = start + delta * (min(float(chunk_metres), distance) / distance)
    if not graph.segment_clear(start, endpoint, obstacles, obstacle_radius=clearance):
        return None
    return [tuple(start), tuple(endpoint)]


def smooth_exploration_bearing(previous, requested, max_step_deg=15.0):
    """Limit frontier steering changes so short replans do not zig-zag."""
    requested = float(requested)
    if previous is None:
        return requested
    delta = (requested - float(previous) + math.pi) % (2.0 * math.pi) - math.pi
    step = math.radians(max_step_deg)
    return float(previous) + float(np.clip(delta, -step, step))


def exploration_bearing(frontier, traversability, width, fov_deg, num_bins=16):
    """Select a drivable frontier sector, faithful to the WildOS additive
    frontier scoring (frontier + goal-heading + reachability) instead of the
    old single max-column heuristic. Returns a bearing relative to forward."""
    front = np.asarray(frontier, dtype=float) if frontier is not None else None
    trav_raw = np.asarray(traversability, dtype=float) if traversability is not None else None
    if front is None:
        return 0.0
    fconf = explore.frontier_conf(front, trav_raw)
    heading, _, _ = explore.best_exploration_heading(fconf, trav_raw, num_bins=num_bins)
    return float(heading)


def mask_bearing(mask, fov_deg):
    """Return the horizontal bearing of a visible semantic mask centroid."""
    if mask is None:
        return None
    ys, xs = np.nonzero(np.asarray(mask, dtype=bool))
    if len(xs) == 0:
        return None
    width = int(np.asarray(mask).shape[1])
    center_x = float(xs.mean())
    return float(np.deg2rad((center_x / max(width, 1) - 0.5) * fov_deg))


def mask_lidar_diagnostics(cloud_world, mask, K, R_wc, camera_position):
    """Count LiDAR points projected into a Camera mask for runtime diagnosis."""
    cloud_world = np.asarray(cloud_world, dtype=float)
    if mask is None or cloud_world.ndim != 2 or cloud_world.shape[1] != 3:
        return {"front": 0, "in_image": 0, "in_mask": 0}
    R_cw = np.asarray(R_wc, dtype=float).T
    t_cw = -R_cw @ np.asarray(camera_position, dtype=float).reshape(3)
    points_cam = cloud_world @ R_cw.T + t_cw
    front = points_cam[:, 2] > 0.1
    points_cam = points_cam[front]
    if len(points_cam) == 0:
        return {"front": 0, "in_image": 0, "in_mask": 0}
    pixels = (np.asarray(K, dtype=float) @ points_cam.T).T
    uv = pixels[:, :2] / pixels[:, 2:3]
    h, w = np.asarray(mask).shape
    ui, vi = uv[:, 0].astype(np.int64), uv[:, 1].astype(np.int64)
    inside = (ui >= 0) & (ui < w) & (vi >= 0) & (vi < h)
    in_image = int(inside.sum())
    if not in_image:
        return {"front": int(len(points_cam)), "in_image": 0, "in_mask": 0}
    selected = np.asarray(mask)[vi[inside], ui[inside]].astype(bool)
    return {"front": int(len(points_cam)), "in_image": in_image,
            "in_mask": int(selected.sum())}


def localize_mask_lidar(cloud_world, mask, K, R_wc, camera_position, min_points):
    """Return median-depth LiDAR lock and exact filtering diagnostics."""
    cloud_world = np.asarray(cloud_world, dtype=float)
    if mask is None or cloud_world.ndim != 2 or cloud_world.shape[1] != 3:
        return None, {"front": 0, "in_image": 0, "in_mask": 0, "finite": 0}
    R_cw = np.asarray(R_wc, dtype=float).T
    t_cw = -R_cw @ np.asarray(camera_position, dtype=float).reshape(3)
    points_cam = cloud_world @ R_cw.T + t_cw
    points_cam = points_cam[points_cam[:, 2] > 0.1]
    front = len(points_cam)
    if not front:
        return None, {"front": 0, "in_image": 0, "in_mask": 0, "finite": 0}
    pixels = (np.asarray(K, dtype=float) @ points_cam.T).T
    uv = pixels[:, :2] / pixels[:, 2:3]
    h, w = np.asarray(mask).shape
    ui, vi = uv[:, 0].astype(np.int64), uv[:, 1].astype(np.int64)
    inside = (ui >= 0) & (ui < w) & (vi >= 0) & (vi < h)
    in_image = int(inside.sum())
    if not in_image:
        return None, {"front": front, "in_image": 0, "in_mask": 0, "finite": 0}
    points_cam = points_cam[inside]
    selected = np.asarray(mask)[vi[inside], ui[inside]].astype(bool)
    points_cam = points_cam[selected]
    in_mask = len(points_cam)
    diag = {"front": front, "in_image": in_image, "in_mask": in_mask, "finite": in_mask}
    if in_mask < int(min_points):
        return None, diag
    distances = np.linalg.norm(points_cam, axis=1)
    median_point_cam = points_cam[np.argsort(distances)[len(distances) // 2]]
    point_world = np.asarray(R_wc, dtype=float) @ median_point_cam + np.asarray(camera_position, dtype=float).reshape(3)
    if not np.isfinite(point_world).all():
        diag["finite"] = 0
        return None, diag
    return point_world, diag


def _read_json_line_blocking(proc):
    while True:
        line = proc.stdout.readline()
        if not line:
            raise RuntimeError("vision worker closed")
        line = line.strip()
        if line.startswith("{"):
            return json.loads(line)


def read_json_line(proc, timeout=None):
    if timeout is None:
        return _read_json_line_blocking(proc)
    holder = {}

    def _read():
        try:
            holder["value"] = _read_json_line_blocking(proc)
        except BaseException as exc:  # noqa: BLE001
            holder["error"] = exc

    thread = threading.Thread(target=_read, daemon=True)
    thread.start()
    thread.join(timeout)
    if thread.is_alive():
        raise TimeoutError("no vision worker response within %.0fs" % timeout)
    if "error" in holder:
        raise holder["error"]
    return holder["value"]


class VisionWorker:
    def __init__(self, py, script, ckpt, source):
        self.proc = subprocess.Popen([py, "-s", "-B", str(script),
                                      "--checkpoint-root", str(ckpt), "--source-root", str(source)],
                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     text=True, encoding="utf-8")
        ready = read_json_line(self.proc, timeout=600.0)
        if ready.get("status") != "ready":
            raise RuntimeError("worker not ready: %s" % ready)

    def infer(self, image, query, out_dir, timeout=180.0):
        request = json.dumps({"image": str(image), "query": query, "out": str(out_dir)})
        self.proc.stdin.write(request + "\n")
        self.proc.stdin.flush()
        return read_json_line(self.proc, timeout=timeout)

    def close(self):
        try:
            self.proc.stdin.write("quit\n")
            self.proc.stdin.flush()
        except Exception:
            pass
        self.proc.terminate()


def stdin_loop():
    while not STATE["quit"]:
        try:
            text = input().strip()
        except EOFError:
            return
        if text:
            STATE["target"] = text
            progress("new semantic target: '%s'" % text)


def third_person(bng, car):
    try:
        modes = bng.camera.get_player_modes(car)
        names = modes.get("modes", modes) if isinstance(modes, dict) else modes
        for name in ("chase", "orbit", "relative", "droneChase"):
            if name in names:
                bng.camera.set_player_mode(car, name, {})
                return name
    except Exception as exc:
        progress("camera set failed: %s" % exc)
    return None


def drive_segment(car, bng, path, seconds, speed, chunk_metres):
    """Let BeamNG's AI follow the polyline; returns metres actually moved.

    Matches the verified project method (beamng_pointgoal.py, pointgoal_kit):
    the AI owns propulsion/transmission via set_line, with NO manual gear and
    NO traffic mode. cling stays False with a constant z so the AI does not
    fight the level geometry.
    """
    if len(path) < 2:
        return 0.0
    car.sensors.poll()
    state = dict(car.state)
    start = np.asarray(state["pos"], float)[:2]
    z = float(np.asarray(state["pos"], float)[2])
    # BeamNG's line tracker is unreliable with long/multi-turn paths. Send only
    # the current pose and one short endpoint, then reacquire and replan.
    endpoint = short_endpoint(path, chunk_metres)

    car.control(throttle=0.0, brake=0.0, parkingbrake=0.0, steering=0.0)
    line = [{"pos": (float(start[0]), float(start[1]), z), "speed": 0.0},
            {"pos": (float(endpoint[0]), float(endpoint[1]), z),
             "speed": min(float(speed), 3.0)}]
    progress("    set_line(2/%d pts, %.1fm) speed=%.1f" % (
        len(path), float(np.linalg.norm(endpoint - start)), min(float(speed), 3.0)))
    if not getattr(car, "_wildos_ai_line_ready", False):
        progress("    enabling BeamNG traffic line tracker")
        car.ai.set_mode("traffic")
        car._wildos_ai_line_ready = True
    car.ai.set_line(line, cling=False)
    bng.control.resume()
    deadline = time.monotonic() + seconds
    last_log = time.monotonic()
    while time.monotonic() < deadline:
        car.sensors.poll()
        if time.monotonic() - last_log >= 1.0:
            p = np.asarray(dict(car.state)["pos"], float)[:2]
            progress("    driving... moved %.1fm" % float(np.linalg.norm(p - start)))
            last_log = time.monotonic()
        time.sleep(0.05)
    car.sensors.poll()
    end = np.asarray(dict(car.state)["pos"], float)[:2]
    return float(np.linalg.norm(end - start))


def unstick(car, bng, pos, heading):
    """Nudge sideways with an AI line to break free; no reverse/gear guessing."""
    side = np.array([-heading[1], heading[0]])
    p = np.asarray(pos, float)
    tgt = p[:2] + side * 4.0
    car.control(throttle=0.0, brake=0.0, parkingbrake=0.0, steering=0.0)
    line = [{"pos": (float(p[0]), float(p[1]), float(p[2])), "speed": 0.0},
            {"pos": (float(tgt[0]), float(tgt[1]), float(p[2])), "speed": 1.5}]
    car.ai.set_mode("traffic")
    car.ai.set_line(line, cling=False)
    bng.control.resume()
    time.sleep(2.0)
    bng.control.pause()


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--home", type=Path, default=DEFAULT_HOME)
    p.add_argument("--user", type=Path, default=DEFAULT_USER)
    p.add_argument("--map-zip", type=Path, default=DEFAULT_ZIP)
    p.add_argument("--spawn", default="spawns_sawmill")
    p.add_argument("--min-lidar-points", type=int, default=20)
    p.add_argument("--start-offset", default="0,0",
                   help="x,y metres added to the spawn transform to avoid a wedged spawn point")
    p.add_argument("--start-z", type=float, default=0.0,
                   help="Metres added to spawn z so the car drops free of overlapping geometry")
    p.add_argument("--query", default="house")
    p.add_argument("--checkpoint-root", type=Path, default=DEFAULT_CKPT)
    p.add_argument("--source-root", type=Path, default=DEFAULT_SOURCE)
    p.add_argument("--worker-script", type=Path, default=ROOT / "vocabulary_query" / "vision_worker.py")
    p.add_argument("--worker-python", default=DEFAULT_WORKER_PYTHON,
                   help="Interpreter for the vision worker (needs torch/transformers), default platform-specific")
    p.add_argument("--binary", default=DEFAULT_BINARY, help="BeamNG executable relative to home")
    p.add_argument("--gfx", default=DEFAULT_GFX, choices=(None, "dx11", "vk"),
                   help="BeamNG rendering API (vk on Linux, None on Windows)")
    p.add_argument("--camera-size", default="960x640",
                   help="Camera resolution WxH; intrinsics are derived from it")
    p.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    p.add_argument("--port", type=int, default=64256)
    p.add_argument("--timeout", type=float, default=120)
    p.add_argument("--max-cycles", type=int, default=30)
    p.add_argument("--drive-seconds", type=float, default=6.0)
    p.add_argument("--success-radius", type=float, default=4.0)
    p.add_argument("--speed", type=float, default=8.0)
    p.add_argument("--explore-metres", type=float, default=35.0)
    p.add_argument("--target-confirmations", type=int, default=3)
    p.add_argument("--target-max-jump", type=float, default=8.0)
    p.add_argument("--target-miss-tolerance", type=int, default=2)
    p.add_argument("--target-stall-limit", type=int, default=3)
    p.add_argument("--motion-chunk-metres", type=float, default=2.0)
    p.add_argument("--headless", action="store_true", help="BeamNG headless (no window)")
    p.add_argument("--draw-path", action="store_true", help="Draw debug path in BeamNG")
    p.add_argument("--save-artifacts", action="store_true", help="Keep per-cycle images, clouds and worker maps")
    p.add_argument("--vision-period", type=int, default=2, help="Run semantic vision every N cycles")
    p.add_argument("--explore-bins", type=int, default=16, help="WildOS angular bins for frontier exploration")
    p.add_argument("--steer-sign", type=float, default=1.0)
    args = p.parse_args(argv)
    if args.success_radius <= 0:
        p.error("--success-radius must be > 0")
    if args.target_confirmations < 1:
        p.error("--target-confirmations must be >= 1")
    if args.target_max_jump <= 0:
        p.error("--target-max-jump must be > 0")
    if args.target_miss_tolerance < 0:
        p.error("--target-miss-tolerance must be >= 0")
    if args.target_stall_limit < 1:
        p.error("--target-stall-limit must be >= 1")
    if args.motion_chunk_metres <= 0:
        p.error("--motion-chunk-metres must be > 0")
    if args.explore_bins < 4:
        p.error("--explore-bins must be >= 4")
    return args


def main(argv=None):
    a = parse_args(argv)
    from beamngpy import BeamNGpy, Scenario, Vehicle
    from beamngpy.connection import Connection

    STATE["target"] = a.query
    STATE["last"] = time.monotonic()
    progress("target = '%s'  (type a new target + Enter any time to change)" % STATE["target"])
    if not a.headless and sys.stdin.isatty():
        threading.Thread(target=stdin_loop, daemon=True).start()
    threading.Thread(target=watchdog, daemon=True).start()

    width, height = (int(v) for v in a.camera_size.lower().split("x"))
    K, h_fov_deg = camera_intrinsics((width, height))
    progress("camera %dx%d FOV_y=%.0f deg -> fx=%.1f cx=%.1f cy=%.1f h_fov=%.1f deg" % (
        width, height, FOV_Y_DEG, K[0, 0], K[0, 2], K[1, 2], h_fov_deg))

    out = a.output / uuid.uuid4().hex[:8]
    out.mkdir(parents=True, exist_ok=True)
    runs_root = ROOT / "vocabulary_query" / "local_runs"
    runs_root.mkdir(parents=True, exist_ok=True)
    run_id = out.name
    spawn = ca.read_spawn(a.map_zip, a.spawn)
    off = [float(v) for v in a.start_offset.split(",")]
    start_pos = [spawn["position"][0] + off[0], spawn["position"][1] + off[1],
                 spawn["position"][2] + a.start_z]
    progress("spawn %s pos=%s start=%s" % (a.spawn, [round(v, 1) for v in spawn["position"]],
                                            [round(v, 1) for v in start_pos]))

    worker = VisionWorker(a.worker_python, a.worker_script, a.checkpoint_root, a.source_root)
    progress("vision worker ready (SigLIP2 + C-RADIO + heads loaded once) via %s" % a.worker_python)

    beamng_kwargs = dict(home=str(a.home), user=str(a.user), binary=a.binary,
                         quit_on_close=False, headless=a.headless, socket_timeout=a.timeout)
    if a.gfx:
        beamng_kwargs["gfx"] = a.gfx
    bng = BeamNGpy("127.0.0.1", a.port, **beamng_kwargs)
    car, rig, line_id = None, {}, None
    log = []

    def write_loop_report():
        (out / "loop_report.json").write_text(json.dumps(log, indent=2), encoding="utf-8")

    try:
        progress("closing any running BeamNG (avoid multi-instance conflicts)")
        if IS_WINDOWS:
            subprocess.run(["taskkill", "/F", "/IM", "BeamNG.tech.x64.exe"], capture_output=True)
        else:
            subprocess.run(["pkill", "-f", a.binary], capture_output=True)
        time.sleep(3.0)
        progress("launching local BeamNG (American Road)%s" % (" [headless]" if a.headless else ""))
        bng.open(launch=True, listen_ip="127.0.0.1")
        scene = Scenario("american_road", "wildos_" + uuid.uuid4().hex, description="local WildOS loop")
        car = Vehicle("wildos_ego", model="pickup", part_config=ca.PART_CONFIG)
        scene.add_vehicle(car, pos=tuple(start_pos), rot_quat=tuple(spawn["rot_quat"]),
                          safe_spawn=True)
        scene.make(bng)
        bng.scenario.load(scene, connect_player_vehicle=False, connect_existing_vehicles=False)
        endpoint = bng.vehicles.start_connection(car, car.extensions)
        car.connection = Connection("127.0.0.1", int(endpoint["result"]),
                                    timeout=min(float(a.timeout), 60.0))
        car.connect(bng)
        bng.scenario.start()
        car.ai.set_mode("disabled")
        car.set_shift_mode("realistic_automatic")
        car.control(throttle=0, brake=1, parkingbrake=1, steering=0, gear=0)
        bng.control.resume()
        time.sleep(3.0)
        try:
            bng.env.set_tod(tod=0.0, play=False)
            progress("time of day fixed at noon; automatic time progression disabled")
        except Exception as exc:
            progress("set_tod failed: %s" % exc)
        if not a.headless:
            progress("third-person camera: %s" % third_person(bng, car))
        rig = ca.make_rig(bng, car, resolution=(width, height), lidar_resolution=32,
                          lidar_max_distance=60, lidar_density=10)
        time.sleep(1.5)

        last_target_xy = None
        last_front = None
        last_trav = None
        last_mask_pixels = 0
        target_track = {"candidate": None, "count": 0, "confirmed": None,
                        "misses": 0, "observations": []}
        position_history = []
        previous_explore_bearing = None
        target_stall_count = 0

        for cycle in range(a.max_cycles):
            target_text = STATE["target"]
            bng.control.pause()
            bng.control.step(1)
            car.sensors.poll()
            pos = np.asarray(dict(car.state)["pos"], float)
            heading = np.asarray(dict(car.state).get("dir", [0, -1, 0]), float)[:2]
            heading = heading / max(float(np.linalg.norm(heading)), 1e-6)

            camera_reading, _, _ = cf.adhoc(rig["camera"], a.timeout)
            image = camera_reading.get("colour")
            cloud_reading, _, _ = cf.adhoc(rig["lidar"], a.timeout)
            raw = np.asarray(cloud_reading.get("pointCloud"), float)
            if raw.ndim != 2 or raw.shape[1] != 3:
                raise RuntimeError("invalid LiDAR cloud shape: %s" % (raw.shape,))
            raw = raw[np.isfinite(raw).all(axis=1)]
            if len(raw) < a.min_lidar_points:
                raise RuntimeError("too few finite LiDAR points: %d" % len(raw))
            image_size = image_dimensions(image)
            if not image_size_matches(image_size, width, height):
                raise RuntimeError("invalid camera image size: %s" % (image_size,))
            world_cloud = ca.cloud_to_world(raw, np.asarray(rig["lidar"].get_position(), float))[0]
            cam_pos = np.asarray(rig["camera"].get_position(), float)
            cam_dir = np.asarray(rig["camera"].get_direction(), float)

            frame = out / ("cycle_%03d" % cycle)
            frame.mkdir(exist_ok=True)
            image_path = frame / "image.png"

            progress("cycle %d: pos=(%.1f,%.1f) query='%s'" % (cycle, pos[0], pos[1], target_text))
            run_vision = cycle % max(1, a.vision_period) == 0 or last_front is None
            if run_vision:
                save_camera_image(image, image_path)
                if a.save_artifacts:
                    np.save(frame / "cloud.npy", world_cloud, allow_pickle=False)
                vout = runs_root / ("%s_c%03d" % (run_id, cycle))
                vres = worker.infer(image_path, target_text, vout)
                mask = select_region_mask(vout / "region_mask.png", vres)
                n_regions = len((vres or {}).get("regions") or [])
                if n_regions > 1:
                    progress("  %d '%s' regions; using largest" % (n_regions, target_text))
                mask_pixels = int(mask.sum()) if mask is not None else 0
                trav = finite_head_map(np.load(vout / "traversability.npy"), "traversability") if (vout / "traversability.npy").is_file() else None
                front = finite_head_map(np.load(vout / "frontier.npy"), "frontier") if (vout / "frontier.npy").is_file() else None
                last_front, last_trav, last_mask_pixels = front, trav, mask_pixels
                if not a.save_artifacts:
                    shutil.rmtree(vout, ignore_errors=True)
            else:
                mask = None
                mask_pixels = last_mask_pixels
                trav, front = last_trav, last_front

            rotation, _ = cf.world_to_camera(cam_pos, cam_dir)
            target_xy = last_target_xy if not run_vision else None
            candidate_xy = None
            lidar_diag = mask_lidar_diagnostics(world_cloud, mask, K, rotation.T, cam_pos)
            if run_vision and mask is not None and mask_pixels > 0:
                progress("  lidar mask points=%d/%d in_image=%d" % (
                    lidar_diag["in_mask"], lidar_diag["front"], lidar_diag["in_image"]))
                lock, lidar_diag = localize_mask_lidar(
                    world_cloud, mask, K, rotation.T, cam_pos, a.min_lidar_points)
                if lock is not None:
                    candidate_xy = (float(lock[0]), float(lock[1]))
                else:
                    progress("  lidar lock rejected: in_mask=%d finite=%d min=%d" % (
                        lidar_diag["in_mask"], lidar_diag["finite"], a.min_lidar_points))
            if run_vision:
                target_xy = target_track_update(
                    target_track, candidate_xy, a.target_max_jump, a.target_confirmations,
                    a.target_miss_tolerance)
                last_target_xy = target_xy

            if target_xy is not None:
                goal = target_xy
            else:
                visual_bearing = mask_bearing(mask, h_fov_deg) if mask_pixels > 0 else None
                if visual_bearing is not None:
                    requested_bearing = visual_bearing
                    progress("  visual fallback bearing=%.1f deg" % math.degrees(visual_bearing))
                else:
                    requested_bearing = exploration_bearing(
                        front, trav, front.shape[1] if front is not None else 1, h_fov_deg,
                        num_bins=a.explore_bins)
                stagnating = len(position_history) == 5 and float(
                    np.linalg.norm(np.ptp(np.asarray(position_history), axis=0))) < 2.0
                if stagnating:
                    progress("  exploration recovery: force current heading")
                    requested_bearing = 0.0
                bearing = smooth_exploration_bearing(previous_explore_bearing, requested_bearing)
                previous_explore_bearing = bearing
                c, s = np.cos(bearing), np.sin(bearing)
                fwd = np.array([heading[0] * c - heading[1] * s, heading[0] * s + heading[1] * c])
                goal = (pos[0] + fwd[0] * a.explore_metres, pos[1] + fwd[1] * a.explore_metres)

            radius_xy = np.linalg.norm(world_cloud[:, :2] - pos[:2], axis=1)
            near = world_cloud[radius_xy < 45.0]
            ground = float(np.percentile(near[:, 2], 5)) if len(near) else float(np.percentile(world_cloud[:, 2], 5))
            obstacle_height = near[:, 2] - ground
            obstacles = near[(obstacle_height > 0.5) & (obstacle_height < 2.5)][:, :2]
            obstacles = obstacles[np.linalg.norm(obstacles - pos[:2], axis=1) > 3.0]
            grid = planner.build_grid(obstacles, (pos[0], pos[1]), goal, half_extent=40.0,
                                      resolution=0.5, inflation=1.8)
            path = planner.simplify(planner.astar(grid, (pos[0], pos[1]), goal), obstacles,
                                    obstacle_radius=1.8)
            distance = float(np.linalg.norm(pos[:2] - np.asarray(goal, float)))
            trav_note = None if trav is None else "%0.3f" % float(trav[:, trav.shape[1] // 2].mean())
            progress("  mask_px=%d target=%s path=%d dist=%.1f trav_center=%s" % (
                mask_pixels, None if target_xy is None else [round(v, 1) for v in target_xy],
                len(path), distance, trav_note))
            target_bearing = None if target_xy is None else bearing_deg(np.asarray(target_xy) - pos[:2])
            path_bearing = None if len(path) < 2 else bearing_deg(np.asarray(path[1]) - pos[:2])
            log.append({"cycle": cycle, "pos": pos[:2].tolist(), "target_text": target_text,
                        "mask_pixels": mask_pixels, "target_xy": target_xy, "goal": list(goal),
                        "candidate_xy": None if candidate_xy is None else list(candidate_xy),
                        "target_confirmations": target_track["count"],
                        "target_misses": target_track["misses"],
                        "target_bearing_deg": target_bearing,
                        "path_bearing_deg": path_bearing,
                        "heading_bearing_deg": bearing_deg(heading),
                        "lidar_diagnostics": lidar_diag,
                        "path": len(path), "distance_m": distance,
                        "traversability_range": None if trav is None else [float(trav.min()), float(trav.max())],
                        "frontier_range": None if front is None else [float(front.min()), float(front.max())]})
            write_loop_report()
            position_history.append(pos[:2].copy())
            if len(position_history) > 5:
                position_history.pop(0)
            if len(position_history) == 5:
                local_span = float(np.linalg.norm(np.ptp(np.asarray(position_history), axis=0)))
                if local_span < 2.0:
                    progress("  stagnation: 5-cycle position span %.2fm" % local_span)

            if target_xy is not None and distance <= a.success_radius:
                progress("TARGET REACHED at %s" % [round(v, 1) for v in target_xy])
                break
            if not path and target_xy is None:
                short = a.explore_metres * 0.4
                fwd = np.array([heading[0], heading[1]])
                goal = (pos[0] + fwd[0] * short, pos[1] + fwd[1] * short)
                grid = planner.build_grid(obstacles, (pos[0], pos[1]), goal, half_extent=40.0,
                                          resolution=0.5, inflation=1.8)
                path = planner.simplify(planner.astar(grid, (pos[0], pos[1]), goal), obstacles,
                                        obstacle_radius=1.8)
            if not path:
                progress("  no path; holding")
                bng.control.resume()
                time.sleep(1.0)
                continue

            if target_xy is not None and target_stall_count > 0:
                direct_path = direct_target_path(
                    pos[:2], target_xy, obstacles, a.motion_chunk_metres, clearance=1.8)
                if direct_path is not None:
                    progress("  approach recovery: direct target short segment")
                    path = direct_path

            if a.draw_path and line_id is not None:
                try:
                    bng.debug.remove_polyline(line_id)
                except Exception:
                    pass
            if a.draw_path:
                line_id = bng.debug.add_polyline([(float(x), float(y), ground + 0.4) for x, y in path],
                                                 (0.1, 1.0, 0.1, 1.0), cling=True)
            bng.control.resume()
            moved = drive_segment(car, bng, path, a.drive_seconds, a.speed, a.motion_chunk_metres)
            bng.control.pause()
            log[-1]["moved_m"] = moved
            after_state = dict(car.state)
            after_pos = np.asarray(after_state["pos"], float)[:2]
            if target_xy is not None:
                distance_after = float(np.linalg.norm(after_pos - np.asarray(target_xy, float)))
                progress_delta = float(distance - distance_after)
                log[-1]["distance_after_m"] = distance_after
                log[-1]["target_progress_m"] = progress_delta
                if progress_delta <= 0.1:
                    target_stall_count += 1
                    progress("  target progress %.2fm (stall %d/%d)" % (
                        progress_delta, target_stall_count, a.target_stall_limit))
                else:
                    target_stall_count = 0
                    progress("  target progress +%.2fm" % progress_delta)
                if target_stall_count >= a.target_stall_limit:
                    progress("TARGET APPROACH STALLED; stopping safely")
                    write_loop_report()
                    break
            write_loop_report()
            if target_xy is None and moved < 1.0:
                progress("  stuck (moved %.2fm); repositioning" % moved)
                unstick(car, bng, pos, heading)

            if not a.save_artifacts:
                shutil.rmtree(frame, ignore_errors=True)

        (out / "loop_report.json").write_text(json.dumps(log, indent=2), encoding="utf-8")
        progress("loop done, report=%s" % (out / "loop_report.json"))
    finally:
        try:
            write_loop_report()
        except Exception:
            pass
        STATE["quit"] = True
        worker.close()
        try:
            if car is not None:
                car.control(throttle=0, brake=1, parkingbrake=1, steering=0, gear=0)
        except Exception:
            pass
        for sensor in reversed(list(rig.values())):
            try:
                sensor.remove()
            except Exception:
                pass
        bng.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
