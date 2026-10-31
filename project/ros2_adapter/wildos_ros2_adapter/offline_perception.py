"""Offline WildOS perception check for a saved ROS2 adapter sample."""
from __future__ import annotations

import argparse
import importlib.util
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
from PIL import Image

from wildos_ros2_adapter.camera import optical_rotation_from_body


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise ImportError(path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def quaternion_matrix(q):
    x, y, z, w = (float(v) for v in q)
    n = x * x + y * y + z * z + w * w
    if n <= 1e-12:
        raise ValueError("zero quaternion")
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - yy - zz, xy - wz, xz + wy],
        [xy + wz, 1.0 - xx - zz, yz - wx],
        [xz - wy, yz + wx, 1.0 - xx - yy],
    ])


def localize_mask_lidar(cloud_world, mask, K, R_wc, t_wc, min_points=20):
    cloud_world = np.asarray(cloud_world, dtype=float)
    R_cw = np.asarray(R_wc, dtype=float).T
    t_cw = -R_cw @ np.asarray(t_wc, dtype=float).reshape(3)
    points_cam = cloud_world @ R_cw.T + t_cw
    points_cam = points_cam[points_cam[:, 2] > 0.1]
    if len(points_cam) == 0:
        return None
    pixels = (np.asarray(K, dtype=float) @ points_cam.T).T
    uv = pixels[:, :2] / pixels[:, 2:3]
    h, w = np.asarray(mask).shape
    ui, vi = uv[:, 0].astype(np.int64), uv[:, 1].astype(np.int64)
    inside = (ui >= 0) & (ui < w) & (vi >= 0) & (vi < h)
    if not inside.any():
        return None
    points_cam = points_cam[inside]
    selected = np.asarray(mask)[vi[inside], ui[inside]].astype(bool)
    points_cam = points_cam[selected]
    if len(points_cam) < min_points:
        return None
    distances = np.linalg.norm(points_cam, axis=1)
    point = points_cam[np.argsort(distances)[len(distances) // 2]]
    return np.asarray(R_wc, dtype=float) @ point + np.asarray(t_wc, dtype=float)


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("sample", type=Path)
    p.add_argument("--query", default="house")
    p.add_argument("--vision-dir", type=Path, default=Path("/home/ly/wildos-opt/vocabulary_query"))
    p.add_argument("--checkpoint-root", type=Path, default=Path("/home/ly/ckpts"))
    p.add_argument("--source-root", type=Path, default=Path("/home/ly/wildos-sim/wildos_port/repro_check/src/nebula2-wildos"))
    p.add_argument("--worker-python", default="/home/ly/vision-cu128/bin/python")
    p.add_argument("--output-dir", type=Path, default=None)
    p.add_argument("--threshold", type=float, default=None)
    args = p.parse_args(argv)
    sample = args.sample.expanduser().resolve()
    image = sample / "image.png"
    cloud_path = sample / "cloud_map.npy"
    camera_info = json.loads((sample / "camera_info.json").read_text())
    tf = json.loads((sample / "tf.json").read_text())
    if "error" in tf.get("ego_front_cam", {}):
        raise RuntimeError("sample has no camera TF")
    if not image.is_file() or not cloud_path.is_file():
        raise FileNotFoundError("sample needs image.png and cloud_map.npy")
    out = (args.output_dir.expanduser().resolve() if args.output_dir else
           args.vision_dir.expanduser().resolve() / "offline_capture_check")
    out.mkdir(parents=True, exist_ok=True)
    for child in out.iterdir():
        if child.is_file():
            child.unlink()
    worker_script = args.vision_dir.expanduser().resolve() / "vision_worker.py"
    proc = subprocess.Popen(
        [args.worker_python, "-s", "-B", str(worker_script),
         "--checkpoint-root", str(args.checkpoint_root),
         "--source-root", str(args.source_root)],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        text=True,
        encoding="utf-8",
    )

    def read_json_line():
        for line in proc.stdout:
            line = line.strip()
            if line.startswith("{"):
                return json.loads(line)
        raise RuntimeError("vision worker closed")

    ready = read_json_line()
    if ready.get("status") != "ready":
        raise RuntimeError(f"vision worker not ready: {ready}")
    proc.stdin.write(json.dumps({"image": str(image), "query": args.query, "out": str(out)}) + "\n")
    proc.stdin.flush()
    result = read_json_line()
    mask_path = out / "region_mask.png"
    mask = np.asarray(Image.open(mask_path).convert("L")) > 127
    cloud = np.load(cloud_path)
    K = np.array(camera_info["k"], dtype=float).reshape(3, 3)
    cam_tf = tf["ego_front_cam"]
    R_wc = optical_rotation_from_body(
        quaternion_matrix(cam_tf["rotation_xyzw"]))
    t_wc = np.asarray(cam_tf["translation"], dtype=float)
    lock = localize_mask_lidar(cloud, mask, K, R_wc, t_wc, min_points=20)
    report = {
        "sample": str(sample),
        "query": args.query,
        "image_size": list(Image.open(image).size),
        "mask_pixels": int(mask.sum()),
        "regions": result.get("regions", []),
        "lidar_points": int(len(cloud)),
        "target_world_xyz": None if lock is None else [float(v) for v in lock],
        "camera_calibration": camera_info.get("calibration_status"),
        "status": "localized" if lock is not None else "mask_without_lidar_lock",
    }
    (out / "offline_capture_report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    try:
        proc.stdin.write("quit\n")
        proc.stdin.flush()
    except Exception:
        pass
    proc.terminate()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
