"""Compare closed-map replay outputs; gtest exit alone is not planning success."""
import csv
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def metrics(mode, index):
    folder = ROOT / ('results_' + mode)
    csv_path = folder / f'case{index}_recorded_smooth.csv'
    failure = folder / f'case{index}_recorded_smooth.error'
    if not csv_path.exists():
        assert failure.exists(), (mode, index, 'missing result')
        return {'success': False, 'reason': failure.read_text().strip()}
    assert not failure.exists(), (mode, index, 'ambiguous stale outputs')
    with csv_path.open() as stream:
        poses = [(float(r['x']), float(r['y']), float(r['yaw']))
                 for r in csv.DictReader(stream)]
    assert len(poses) > 1 and all(math.isfinite(v) for p in poses for v in p)
    edges = [(b[0]-a[0], b[1]-a[1]) for a, b in zip(poses, poses[1:])]
    edges = [e for e in edges if math.hypot(*e) > 1e-9]
    length = sum(math.hypot(*e) for e in edges)
    cusps = sum(a[0]*b[0] + a[1]*b[1] < 0 for a, b in zip(edges, edges[1:]))
    return {'success': True, 'length': length, 'cusps': cusps,
            'cost': length + 3*cusps, 'start': poses[0], 'end': poses[-1],
            'sha256': hashlib.sha256(csv_path.read_bytes()).hexdigest()}


def main():
    rows = [{'case': i, **{mode: metrics(mode, i) for mode in ('baseline', 'final')}}
            for i in range(19)]
    for row in rows:
        before, after = row['baseline'], row['final']
        assert before['success'] == after['success'], row
        if after['success']:
            assert before['start'] == after['start'], row
            assert after['cost'] <= before['cost'] + 1e-6, row
    comparable = [r for r in rows if r['final']['success']]
    summary = {
        'cases': len(rows), 'both_succeeded': len(comparable),
        'both_failed': [r['case'] for r in rows if not r['final']['success']],
        'lower_cost': [r['case'] for r in comparable
                       if r['final']['cost'] < r['baseline']['cost'] - 1e-6],
        'fewer_cusps': [r['case'] for r in comparable
                        if r['final']['cusps'] < r['baseline']['cusps']],
        'rows': rows,
        'scope': 'Reconstructed public maps and published path endpoints; not exact private requests or physical arrival.'}
    (ROOT / 'final_comparison.json').write_text(json.dumps(summary, indent=2), encoding='utf8')
    print(json.dumps({k: v for k, v in summary.items() if k != 'rows'}))


if __name__ == '__main__':
    main()
