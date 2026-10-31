"""Offline manifest replay. Never acquires, drives, publishes, or invents timestamps."""

import argparse
import json
from pathlib import Path

import numpy as np

from .core import Localizer, Policy, validate_calibration
from .query_io import Invalid, fields, load_query, read_json, require, sha256


def run(path):
    manifest = read_json(path)
    fields(manifest, "schema status unresolved calibration policy frames")
    require(manifest["schema"] == "target-offline-manifest/1", "Wrong manifest schema")
    require(manifest["status"] in ("DRAFT", "VERIFIED"), "Unknown manifest status")
    require(type(manifest["unresolved"]) is list and all(type(s) is str and s.strip() for s in manifest["unresolved"]),
            "Unresolved must be a list of nonempty strings")
    if manifest["status"] == "DRAFT":
        result = Localizer({}).handoff("draft_manifest")
        result["unresolved"] = manifest["unresolved"] or ["manifest_not_verified"]
        return [result]
    require(not manifest["unresolved"], "Verified manifest still has unresolved evidence")
    base = Path(path).resolve().parent

    def artifact(value):
        require(type(value) is str and bool(value.strip()), "Explicit artifact path required")
        return base / value

    cal = read_json(artifact(manifest["calibration"]))
    validate_calibration(cal)
    require(type(manifest["policy"]) is dict, "Policy object required")
    engine = Localizer(cal, Policy(**manifest["policy"]))
    require(type(manifest["frames"]) is list, "Frame entries required")
    if not manifest["frames"]:
        return [engine.handoff("no_acquired_frames")]
    outputs = []
    for entry in manifest["frames"]:
        try:
            fields(entry, "query_result prefix image cloud frame")
            query = load_query(artifact(entry["query_result"]), entry["prefix"])
            frame = read_json(artifact(entry["frame"]))
            require(frame["image"]["sha256"] == sha256(artifact(entry["image"])), "Image file digest mismatch")
            # JSON sidecar cannot silently carry competing inline point data.
            require(frame["cloud"]["points_xyz_m"] is None, "CLI frame points must be null; use explicit cloud artifact")
            frame["cloud"]["points_xyz_m"] = np.load(artifact(entry["cloud"]), allow_pickle=False)
            outputs.append(engine.process(query, frame))
        except (OSError, ValueError, TypeError, KeyError, OverflowError) as exc:
            engine.release("LOST" if engine.key is not None else "SEARCHING")
            outputs.append(engine.handoff(str(exc)))
    return outputs


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=Path(__file__).with_name("manifest.draft.json"))
    args = parser.parse_args(argv)
    try:
        outputs = run(args.manifest)
    except (OSError, ValueError, TypeError, KeyError, OverflowError) as exc:
        outputs = [Localizer({}).handoff(str(exc))]
    for result in outputs:
        print(json.dumps(result, ensure_ascii=True, allow_nan=False))
    return 2 if any(result["status"] == "waiting" for result in outputs) else 0


if __name__ == "__main__":
    raise SystemExit(main())
