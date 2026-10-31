"""End-to-end driver: query captured frames, strictly validate them, then localize.

Two input modes:
  --capture DIR       legacy single capture dir (mounted/static/lidar manifests)
  --frames-dir DIR    synchronized frames produced by integration.capture_frames
                      (DIR/frame_NNN/{image.png,cloud.npy,capture.json})

Real model inference runs in a separate process (vocabulary_query/query.py) so the
offline/network guarantees of that tool are untouched. This driver only orchestrates
and validates; it never invents a synchronized acquisition stamp.
"""
import argparse
import json
import subprocess
import sys
from pathlib import Path

from target_localization.core import Localizer, Policy
from target_localization.query_io import Invalid, load_query

from . import build_frame, common, make_calibration


def make_policy(args):
    overrides = {}
    if args.max_extent_m is not None:
        overrides["max_extent_m"] = args.max_extent_m
    if args.max_depth_span_m is not None:
        overrides["max_depth_span_m"] = args.max_depth_span_m
    if args.min_coverage is not None:
        overrides["min_coverage"] = args.min_coverage
    if args.max_ring_same_depth_fraction is not None:
        overrides["max_ring_same_depth_fraction"] = args.max_ring_same_depth_fraction
    if args.alpha_silhouette is not None:
        overrides["alpha_silhouette"] = args.alpha_silhouette
    if args.ttl_ns is not None:
        overrides["ttl_ns"] = args.ttl_ns
    return Policy(**overrides) if overrides else None


def run_query(args, run_dir, image):
    script = args.query_script.resolve()
    if not script.is_file():
        raise FileNotFoundError(script)
    command = [str(args.query_python), "-B", str(script),
               "--source-root", str(args.source_root.resolve()),
               "--checkpoint-root", str(args.checkpoint_root.resolve()),
               "--image", str(Path(image).resolve()),
               "--output", str(run_dir)]
    for query in args.query:
        command += ["--query", query]
    if args.threshold is not None:
        command += ["--threshold", str(args.threshold)]
    completed = subprocess.run(command, text=True, capture_output=True)
    result_path = run_dir / "result.json"
    if not result_path.is_file():
        raise RuntimeError("query.py produced no result.json\nstdout:\n%s\nstderr:\n%s"
                           % (completed.stdout, completed.stderr))
    report = common.read_json(result_path)
    if report.get("status") != "completed" or report.get("inference_ran") is not True:
        raise RuntimeError("query.py did not complete inference: %s" % report.get("status"))
    return report


def public_regions(query):
    return [{"box_xyxy": list(region["box_xyxy"]), "area_pixels": int(region["area_pixels"]),
             "mean_cosine": float(region["mean_cosine"]), "max_cosine": float(region["max_cosine"])}
            for region in query["regions"]]


def query_and_load(args, run_dir, image):
    run_dir = Path(run_dir)
    if args.reuse_query:
        effective = Path(args.reuse_query)
        report = common.read_json(effective / "result.json")
    else:
        if run_dir.exists() and any(run_dir.iterdir()):
            raise RuntimeError("query run dir exists and is not empty: %s" % run_dir)
        report = run_query(args, run_dir, image)
        effective = run_dir
    loaded = [{"prefix": "query_%03d" % i, "query": load_query(effective / "result.json", "query_%03d" % i)}
              for i in range(len(args.query))]
    return effective, loaded, report


def legacy(args, out):
    image = args.image.resolve() if args.image else (args.capture / "static.png").resolve()
    capture = common.load_capture(args.capture)
    epoch = args.capture.resolve().name
    run_dir = args.query_script.resolve().parent / ("integration_" + epoch)
    run_dir, loaded, query_report = query_and_load(args, run_dir, image)
    report = {"schema": "integration-e2e/1", "mode": "legacy_capture",
              "capture": str(args.capture.resolve()), "image": str(image), "epoch": epoch,
              "queries": args.query, "stages": {"query": {"dir": str(run_dir),
              "status": query_report.get("status"), "torch": query_report.get("torch")}},
              "blockers": []}
    report["stages"]["load_query"] = {"validated": len(loaded), "requested": len(args.query)}
    calibration = make_calibration.build(args.accept_calibration, capture)
    make_calibration.write(out / "calibration.json", calibration)
    block = build_frame.timing_block(capture)
    sync = build_frame.synchronization(capture)
    report["stages"]["calibration"] = {"status": calibration["status"]}
    report["stages"]["synchronization"] = sync
    if block:
        report["blockers"].append(block)
        report["stages"]["localization"] = {"mode": "refused", "reason": block}
    else:
        engine = Localizer(calibration, make_policy(args))
        results = []
        for entry in loaded:
            frame = build_frame.build_frame(capture, image, entry["query"], calibration, epoch)
            results.append(engine.process(entry["query"], frame))
        report["stages"]["localization"] = {"mode": "executed", "results": results}
    report["regions"] = {e["prefix"]: {"query": e["query"]["query"], "regions": public_regions(e["query"])}
                         for e in loaded}
    return report


def multi_frame(args, out):
    frames_root = args.frames_dir.resolve()
    frame_dirs = sorted(p.parent for p in frames_root.glob("frame_*/capture.json"))
    if not frame_dirs:
        raise RuntimeError("no frame_*/capture.json under %s" % frames_root)
    records = {frame_dir: common.read_json(frame_dir / "capture.json") for frame_dir in frame_dirs}
    sources = {record.get("time_source") for record in records.values()}
    if len(sources) != 1:
        raise RuntimeError("inconsistent capture time sources: %s" % sorted(sources))
    epoch = frames_root.parent.name
    calibration = make_calibration.build(args.accept_calibration, None, time_source=sources.pop())
    make_calibration.write(out / "calibration.json", calibration)
    engine = Localizer(calibration, make_policy(args))
    report = {"schema": "integration-e2e/1", "mode": "multi_frame",
              "frames_dir": str(frames_root), "epoch": epoch, "queries": args.query,
              "stages": {"calibration": {"status": calibration["status"]}}, "blockers": [], "frames": []}
    for frame_dir in frame_dirs:
        entry = {"frame": frame_dir.name, "image": str(frame_dir / "image.png")}
        record = records[frame_dir]
        try:
            run_dir = args.query_script.resolve().parent / ("integration_%s_%s" % (epoch, frame_dir.name))
            run_dir, loaded, _ = query_and_load(args, run_dir, frame_dir / "image.png")
            query = loaded[0]["query"]
            entry["regions"] = public_regions(query)
            frame = build_frame.from_capture_record(record, frame_dir / "image.png", query, calibration)
            entry["handoff"] = engine.process(query, frame)
        except (Invalid, OSError, ValueError, RuntimeError, build_frame.NotSynchronized) as exc:
            entry["error"] = str(exc)
            report["blockers"].append("%s: %s" % (frame_dir.name, exc))
        report["frames"].append(entry)
    states = [f.get("handoff", {}).get("state") for f in report["frames"] if "handoff" in f]
    report["stages"]["localization"] = {"mode": "executed", "states": states,
                                        "confirmed": states.count("CONFIRMED")}
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--capture", type=Path)
    source.add_argument("--frames-dir", type=Path)
    parser.add_argument("--image", type=Path, help="Only for --capture: image the query runs on")
    parser.add_argument("--query", action="append", required=True)
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--checkpoint-root", type=Path, required=True)
    parser.add_argument("--query-script", type=Path,
                        default=Path(__file__).resolve().parent.parent / "vocabulary_query" / "query.py")
    parser.add_argument("--query-python", default=sys.executable)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--threshold", type=float)
    parser.add_argument("--reuse-query", type=Path)
    parser.add_argument("--accept-calibration",
                        help="Reference string that accepts the fixture calibration (marks VERIFIED)")
    parser.add_argument("--max-extent-m", type=float,
                        help="Explicit Policy override for large fixture targets (default keeps strict policy)")
    parser.add_argument("--max-depth-span-m", type=float)
    parser.add_argument("--min-coverage", type=float)
    parser.add_argument("--max-ring-same-depth-fraction", type=float)
    parser.add_argument("--alpha-silhouette", type=float)
    parser.add_argument("--ttl-ns", type=int,
                        help="Evidence lifetime override (default 1s); fixture frames seconds apart need a larger window")
    args = parser.parse_args(argv)
    if args.reuse_query and args.frames_dir:
        parser.error("--reuse-query applies to --capture only (each frame runs its own query)")

    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    report = multi_frame(args, out) if args.frames_dir else legacy(args, out)
    report["status"] = "blocked" if report["blockers"] else "completed"
    (out / "integration_report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    summary = {"status": report["status"], "mode": report["mode"],
               "calibration": report["stages"]["calibration"]["status"],
               "blockers": report["blockers"], "report": str(out / "integration_report.json")}
    print(json.dumps(summary, ensure_ascii=True, indent=2))
    return 2 if report["blockers"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
