"""Emit a target-calibration/1 document from fixture geometry.

Default output is DRAFT: the nominal pinhole K (FOV 60 deg) and the static
world->camera SE3 are derived, not independently accepted. Pass --accept with a
reference string only after a human or independent artifact actually accepts them.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from . import common


def build(accepted_reference=None, capture=None, time_source=None):
    rotation, translation = common.static_world_to_camera()
    if accepted_reference:
        evidence = {"accepted": True, "reference": accepted_reference}
        status = "VERIFIED"
    else:
        evidence = {"accepted": False,
                    "reference": "nominal FOV=60deg K and fixture static world->camera SE3; unaccepted"}
        status = "DRAFT"
    camera_evidence = evidence
    if capture is not None:
        fit = capture["static"]["api_fit"]
        visual = capture["static"]["visual"]
        nominal = fit["nominal_focal"]
        mag_error = max(abs(abs(fit["fx"]) / nominal - 1.0), abs(abs(fit["fy"]) / nominal - 1.0))
        note = ("api_fit |f|/nominal error %.4f, visual detected=%s max_reprojection_px=%s; sign convention negative"
                % (mag_error, visual.get("detected"), visual.get("max_reprojection_px")))
        camera_evidence = {**evidence, "reference": evidence["reference"] + " | " + note}
    return {
        "schema": "target-calibration/1",
        "status": status,
        "evidence": evidence,
        "world_frame": common.WORLD_FRAME,
        "camera_frame": common.CAMERA_FRAME,
        "local_cloud_frame": common.LOCAL_CLOUD_FRAME,
        "time_source": time_source or common.TIME_SOURCE,
        "units": "m",
        "camera": {
            "width": common.WIDTH,
            "height": common.HEIGHT,
            "projection": "rectilinear_pinhole",
            "pixel_center": "integer_centers_half_extent",
            "axes": "x_right_y_down_z_forward",
            "pixels_in_target_optical_frame": True,
            "K": common.nominal_k(),
            "D": [0.0, 0.0, 0.0, 0.0, 0.0],
            "evidence": camera_evidence,
        },
        "local_to_camera": {
            "source_frame": common.LOCAL_CLOUD_FRAME,
            "target_frame": common.CAMERA_FRAME,
            "R": np.asarray(rotation, float).tolist(),
            "translation_m": np.asarray(translation, float).tolist(),
            "units": "m",
            "evidence": evidence,
        },
    }


def write(path, calibration):
    Path(path).write_text(json.dumps(calibration, ensure_ascii=False, indent=2), encoding="utf-8")
    return path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--capture", type=Path, help="Optional capture dir to record api_fit/visual evidence")
    parser.add_argument("--accept", help="Reference string that accepts this calibration (marks VERIFIED)")
    args = parser.parse_args(argv)
    capture = common.load_capture(args.capture) if args.capture else None
    calibration = build(args.accept, capture)
    write(args.output, calibration)
    print(json.dumps({"output": str(args.output), "status": calibration["status"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
