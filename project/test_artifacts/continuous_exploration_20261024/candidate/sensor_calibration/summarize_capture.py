"""Read saved JSON evidence only; no simulator, image loading or dependencies."""
import argparse
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parent


def summarize(capture):
    capture = Path(capture).resolve()
    if not capture.is_dir():
        raise ValueError('Capture directory does not exist: ' + str(capture))
    issues = []

    def read(path, kind):
        if not path.exists():
            return None
        try:
            value = json.loads(path.read_text(encoding='utf-8-sig'))
            if not isinstance(value, kind):
                raise ValueError('unexpected JSON type')
            return value
        except (OSError, ValueError) as exc:
            issues.append(path.relative_to(capture).as_posix() + ': ' + str(exc))
            return None

    report = read(capture / 'runtime_report.json', dict)
    if report is None:
        issues.append('runtime_report.json unavailable; calibration status unknown')
    report = report or {}
    snapshots = []
    preview = capture / 'preview'
    for directory in sorted(preview.iterdir()) if preview.is_dir() else []:
        if not directory.is_dir():
            continue
        sensors = {}
        for name in ('static', 'mounted', 'lidar', 'imu'):
            path = directory / (name + ('_metadata.json' if name == 'imu' else '.json'))
            meta = read(path, dict)
            if meta is None and not (name == 'imu' and (directory / 'imu.json').exists()):
                continue
            meta = meta or {}
            sensor = {key: meta.get(key) for key in (
                'metadata', 'host_acquisition_fps', 'frame_observed', 'display_counts',
                'raw_unfiltered', 'synchronized')}
            if name == 'imu':
                records = read(directory / 'imu.json', list)
                valid = []
                for record in records if records is not None else []:
                    if not isinstance(record, dict):
                        continue
                    vectors = [record.get(key) for key in ('accRaw', 'angVel')]
                    vectors = [[v.get(axis) for axis in ('x', 'y', 'z')] if isinstance(v, dict) else v for v in vectors]
                    if all(isinstance(v, list) and len(v) == 3 and all(
                            isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)
                            for x in v) for v in vectors):
                        valid.append({key: record[key] for key in ('time', 'accRaw', 'angVel') if key in record})
                sensor.update(received_samples=meta.get('received_samples'),
                              saved_samples=meta.get('saved_samples'),
                              actual_record_count=len(records) if records is not None else None,
                              actual_empty=(not records) if records is not None else None,
                              finite_accel_gyro_count=len(valid) if records is not None else None,
                              latest_valid_measurement=valid[-1] if valid else None)
                if records is not None and meta.get('saved_samples') not in (None, len(records)):
                    issues.append(directory.name + ': IMU saved_samples differs from actual list length')
            sensors[name] = sensor
        snapshots.append(dict(snapshot=directory.name, sensors=sensors))
    return dict(capture=str(capture), calibrated=report.get('calibrated'),
                runtime_observed=report.get('runtime_observed'), scope=report.get('scope'),
                diagnostics={key: report.get(key) for key in ('checks', 'error', 'cleanup_errors', 'preview', 'manual_driving', 'demanded')},
                snapshots=snapshots, issues=issues,
                limitations=[
                    'Saved evidence only; no image or point-cloud quality recomputation.',
                    'Missing values are unknown, not zero. No snapshot means S was not saved or evidence is unavailable.',
                    'IMU counts describe saved polls, not cumulative empty polls; preview retains last valid frames.',
                    'Sequential polls may return cached frames; host FPS is not sensor frequency or simulation time.',
                    'No synchronization, verified moving LiDAR extrinsics, navigation readiness or production calibration is established.'])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('capture', nargs='?', type=Path, help='captures/<id> directory')
    source.add_argument('--latest', action='store_true', help='Newest capture by runtime report mtime, else directory mtime')
    parser.add_argument('--output', type=Path, help='Optional summary.json in an existing directory; stdout by default')
    parser.add_argument('--overwrite', action='store_true', help='Explicitly replace an existing summary.json')
    args = parser.parse_args(argv)
    if args.overwrite and args.output is None:
        parser.error('--overwrite requires --output')
    try:
        capture = args.capture
        if args.latest:
            candidates = [p for p in (ROOT / 'captures').iterdir() if p.is_dir()]
            if not candidates:
                raise ValueError('No captures found')
            capture = max(candidates, key=lambda p: ((p / 'runtime_report.json') if (p / 'runtime_report.json').is_file() else p).stat().st_mtime_ns)
        result = summarize(capture)
        # Invalid nonfinite legacy JSON is a diagnostic, never emitted as valid JSON evidence.
        text = json.dumps(result, indent=2, ensure_ascii=True, allow_nan=False) + '\n'
        if args.output:
            if args.output.name != 'summary.json' or not args.output.parent.is_dir():
                raise ValueError('--output must name summary.json in an existing directory')
            with args.output.open('w' if args.overwrite else 'x', encoding='utf-8') as stream:
                stream.write(text)
        else:
            print(text, end='')
    except (OSError, ValueError) as exc:
        parser.exit(1, 'Summary error: ' + str(exc) + '\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
