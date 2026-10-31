"""Offline reproduction of the server's multi-view target triangulation.

Uses only the pulled artifacts under _server_repro (query masks + capture.json
poses), no server, no checkpoints, no clouds. Mirrors run_object_search: particle
triangulation with the same gaussian prior, then compares to the checkerboard
truth (0, -9.999, 2.35).

    python tools/offline_localize.py --run e45b06b2
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from integration import common  # noqa: E402
from object_search import triangulation as T  # noqa: E402

REPRO = ROOT / "_server_repro"
TRUTH = np.array([0.0, -9.999, 2.35])


def load_mask(path):
    import cv2
    image = cv2.imdecode(np.fromfile(str(path), dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(path)
    return image > 127


def camera_to_world(record):
    R_wc = np.asarray(record["pose"]["R"], float).T
    t_wc = -R_wc @ np.asarray(record["pose"]["translation_m"], float)
    return R_wc, t_wc


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--run", default="e45b06b2")
    p.add_argument("--prefix", default="query_000")
    p.add_argument("--num-particles", type=int, default=2000)
    p.add_argument("--hypothesis", type=float, nargs=3, default=[0.0, -10.0, 2.35])
    p.add_argument("--distribution", default="gaussian_hyp")
    p.add_argument("--hypothesis-std", type=float, default=1.0)
    a = p.parse_args(argv)

    frames = REPRO / "capture_out" / a.run / "frames"
    vocab = REPRO / "vision_query" / "vocabulary_query"
    K = np.asarray(common.nominal_k(), float)
    views, locks = [], []
    for frame_dir in sorted(frames.glob("frame_*")):
        record = common.read_json(frame_dir / "capture.json")
        mask_path = vocab / ("integration_%s_%s" % (a.run, frame_dir.name)) / (a.prefix + "_region_mask.png")
        mask = load_mask(mask_path)
        R_wc, t_wc = camera_to_world(record)
        views.append(T.View(mask=mask, K=K, R_wc=R_wc, t_wc=t_wc))
        locks.append({"frame": frame_dir.name, "mask_pixels": int(mask.sum())})

    estimate, method = T.localize(views, cloud_world=None, num_particles=a.num_particles,
                                  depth_range=(1.0, 100.0),
                                  depth_hypothesis={"location": list(a.hypothesis)},
                                  distribution=a.distribution, hypothesis_std=a.hypothesis_std,
                                  method="weighted")
    estimate = np.asarray(estimate, float)
    error = float(np.linalg.norm(estimate - TRUTH))
    server = REPRO / "object_search_out" / "object_search.json"
    server_pos = common.read_json(server).get("triangulated_position") if server.is_file() else None
    report = {"run": a.run, "views": len(views), "method": method, "per_frame": locks,
              "local_estimate": estimate.tolist(),
              "local_error_m": error, "truth": TRUTH.tolist(),
              "server_estimate": server_pos,
              "delta_vs_server_m": (float(np.linalg.norm(estimate - np.asarray(server_pos, float)))
                                    if server_pos else None)}
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
