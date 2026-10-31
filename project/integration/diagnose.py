"""Diagnose why a captured frame's region was rejected by the Localizer projection.

Usage:
  python -m integration.diagnose --frames-dir DIR --frame frame_000 \
      --query-result vocabulary_query/integration_<epoch>_frame_000/result.json \
      --calibration e2e_out/calibration.json
"""
import argparse
import json
from pathlib import Path

import numpy as np

from target_localization.core import Policy, validate_calibration
from target_localization.query_io import load_query

from . import common


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--frames-dir", type=Path, required=True)
    p.add_argument("--frame", default="frame_000")
    p.add_argument("--query-result", type=Path, required=True)
    p.add_argument("--calibration", type=Path, required=True)
    p.add_argument("--prefix", default="query_000")
    args = p.parse_args(argv)

    frame_dir = args.frames_dir.resolve() / args.frame
    record = common.read_json(frame_dir / "capture.json")
    cloud = np.load(frame_dir / "cloud.npy", allow_pickle=False)
    calibration = common.read_json(args.calibration)
    k = validate_calibration(calibration)
    query = load_query(args.query_result.resolve(), args.prefix)

    policy = Policy()
    R = np.asarray(record["pose"]["R"], float)
    t = np.asarray(record["pose"]["translation_m"], float)
    optical = cloud @ R.T + t
    z = optical[:, 2]
    indices = np.flatnonzero((z >= policy.near_m) & (z <= policy.far_m))
    uv = optical[indices, :2] / z[indices, None]
    uv = uv * [k[0, 0], k[1, 1]] + [k[0, 2], k[1, 2]]
    h, w = query["mask"].shape
    inside = (np.isfinite(uv).all(axis=1) & (uv[:, 0] >= -0.5) & (uv[:, 0] < w - 0.5)
              & (uv[:, 1] >= -0.5) & (uv[:, 1] < h - 0.5))
    indices, uv = indices[inside], uv[inside]
    pixels = np.floor(uv + 0.5).astype(np.int64)
    order = np.argsort(z[indices], kind="stable")
    indices, pixels = indices[order], pixels[order]
    _, first = np.unique(pixels[:, 1] * w + pixels[:, 0], return_index=True)
    indices, pixels = indices[first], pixels[first]

    region = query["regions"][0]
    selected = indices[region["mask"][pixels[:, 1], pixels[:, 0]]]
    zs = np.sort(z[selected])
    gaps = np.diff(zs)
    split = np.flatnonzero(gaps > policy.depth_gap_m)
    nearest = (split[0] + 1) if len(split) else len(zs)
    order_local = np.argsort(z[selected], kind="stable")
    nearest_idx = selected[order_local[:nearest]]
    nearest_points = cloud[nearest_idx]
    area = region["area_pixels"]
    print(json.dumps({
        "frame": args.frame,
        "cloud_points": int(len(cloud)),
        "finite": bool(np.isfinite(cloud).all()),
        "region_area_pixels": int(area),
        "region_mean_cosine": float(region["mean_cosine"]),
        "min_mean_cosine": policy.min_mean_cosine,
        "selected_returns": int(len(selected)),
        "nearest_group_size": int(nearest),
        "nearest_coverage": float(nearest / area),
        "min_coverage": policy.min_coverage,
        "min_returns": policy.min_returns,
        "nearest_z_min_m": float(zs[:nearest].min()) if nearest else None,
        "nearest_z_max_m": float(zs[:nearest].max()) if nearest else None,
        "nearest_depth_span_m": float(np.ptp(zs[:nearest])) if nearest else None,
        "max_depth_span_m": policy.max_depth_span_m,
        "nearest_world_extent_m": float(np.linalg.norm(np.ptp(nearest_points, axis=0))) if nearest else None,
        "max_extent_m": policy.max_extent_m,
        "depth_min_m": float(zs.min()) if len(zs) else None,
        "depth_max_m": float(zs.max()) if len(zs) else None,
        "depth_groups": int(len(split) + 1) if len(zs) else 0,
        "group_sizes": [int(n) for n in np.diff(np.concatenate(([0], split, [len(zs)])))] if len(zs) else [],
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
