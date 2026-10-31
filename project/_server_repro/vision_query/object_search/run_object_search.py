"""Run WildOS-style coarse goal localization on captured frames plus query masks.

Reads integration capture-frame/1 records (image/cloud/camera pose) and vocabulary_query
region masks, then reports the LiDAR-in-mask lock per frame and a multi-view particle
triangulation over all frames.

Usage:
  python -m object_search.run_object_search \
    --frames-dir FRAMES --vocab-dir VOCAB --epoch RUNID --prefix query_000 \
    --calibration CAL.json --output OUT
"""
import argparse
import json
from pathlib import Path

import numpy as np

from . import triangulation as T


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def load_mask_png(path):
    import cv2
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(path)
    return image > 127


def camera_to_world(record):
    R_wc = np.asarray(record["pose"]["R"], float).T           # record pose is world->camera
    t_wc = -R_wc @ np.asarray(record["pose"]["translation_m"], float)
    return R_wc, t_wc


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--frames-dir", type=Path, required=True)
    p.add_argument("--vocab-dir", type=Path, required=True)
    p.add_argument("--epoch", required=True)
    p.add_argument("--prefix", default="query_000")
    p.add_argument("--calibration", type=Path, required=True)
    p.add_argument("--num-particles", type=int, default=2000)
    p.add_argument("--depth-min", type=float, default=1.0)
    p.add_argument("--depth-max", type=float, default=100.0)
    p.add_argument("--hypothesis-xyz", type=float, nargs=3, metavar=("X", "Y", "Z"),
                   help="Optional prior location to narrow depth sampling")
    p.add_argument("--distribution", choices=("uniform", "gaussian_hyp", "uniform_hyp"), default="uniform")
    p.add_argument("--hypothesis-std", type=float, default=5.0)
    p.add_argument("--method", choices=("weighted", "minimum"), default="weighted")
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args(argv)

    calibration = read_json(args.calibration)
    K = np.asarray(calibration["camera"]["K"], float)
    views, per_frame = [], []
    for capture in sorted(args.frames_dir.glob("frame_*/capture.json")):
        frame_dir = capture.parent
        record = read_json(capture)
        result = args.vocab_dir / ("integration_%s_%s" % (args.epoch, frame_dir.name)) / "result.json"
        mask_path = result.parent / (args.prefix + "_region_mask.png")
        if not result.is_file() or not mask_path.is_file():
            per_frame.append({"frame": frame_dir.name, "error": "missing query artifacts"})
            continue
        mask = load_mask_png(mask_path)
        R_wc, t_wc = camera_to_world(record)
        cloud = np.load(frame_dir / "cloud.npy", allow_pickle=False)
        views.append(T.View(mask=mask, K=K, R_wc=R_wc, t_wc=t_wc))
        lock = T.localize_in_range(cloud, mask, K, R_wc, t_wc, min_points=150)
        per_frame.append({"frame": frame_dir.name, "lidar_lock": None if lock is None else lock.tolist(),
                          "mask_pixels": int(mask.sum())})

    report = {"schema": "object-search/1", "frames_dir": str(args.frames_dir),
              "epoch": args.epoch, "prefix": args.prefix, "per_frame": per_frame, "views": len(views),
              "distribution": args.distribution, "hypothesis": args.hypothesis_xyz}
    if views:
        hypothesis = None if args.hypothesis_xyz is None else {"location": list(args.hypothesis_xyz)}
        position, method = T.localize(views, cloud_world=None, num_particles=args.num_particles,
                                      depth_range=(args.depth_min, args.depth_max),
                                      depth_hypothesis=hypothesis, distribution=args.distribution,
                                      hypothesis_std=args.hypothesis_std, method=args.method)
        report["triangulated_position"] = position.tolist()
        report["method"] = method
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "object_search.json").write_text(json.dumps(report, ensure_ascii=False, indent=2),
                                                    encoding="utf-8")
    print(json.dumps(report, ensure_ascii=True, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
