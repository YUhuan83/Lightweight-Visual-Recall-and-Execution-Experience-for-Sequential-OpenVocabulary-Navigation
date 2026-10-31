"""JSONL normalized sensor/clock replay, fail closed on the first bad record."""

import argparse
import json
import sys

from .core import AcquisitionClock, SensorAdapter, ValidationError, load_calibration


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", help="JSONL file; '-' for stdin")
    parser.add_argument("--epoch", required=True)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--calibration")
    mode.add_argument("--clock-source", help="explicit normalized acquisition source, not TimeSensor")
    parser.add_argument("--purpose", choices=("navigation", "debug"), default="navigation")
    args = parser.parse_args(argv)
    stream = None
    line_number = 0
    try:
        adapter = (SensorAdapter(load_calibration(args.calibration), args.epoch, purpose=args.purpose)
                   if args.calibration else AcquisitionClock(args.clock_source, args.epoch))
        stream = sys.stdin if args.input == "-" else open(args.input, encoding="utf-8")
        for line_number, line in enumerate(stream, 1):
            record = json.loads(line)
            if not isinstance(record, dict):
                raise ValidationError("record must be an object")
            if record.get("event") == "begin_epoch":
                adapter.begin_epoch(record.get("epoch"))
                result = {"event": "begin_epoch", "epoch": record["epoch"], "flush_consumers": True}
            elif args.calibration:
                result = adapter.adapt(record.get("sample"), matched_transform=record.get("matched_transform"))
            else:
                result = adapter.accept(record.get("seconds"), source=record.get("source"),
                                        epoch=record.get("epoch"), time_kind=record.get("time_kind"))
            print(json.dumps({"line": line_number, "dropped": result is None, "result": result}, allow_nan=False))
        return 0
    except (ValidationError, OSError, ValueError) as exc:
        print(f"replay rejected at line {line_number}: {exc}", file=sys.stderr)
        return 2
    finally:
        if stream is not None and stream is not sys.stdin:
            stream.close()


if __name__ == "__main__":
    sys.exit(main())
