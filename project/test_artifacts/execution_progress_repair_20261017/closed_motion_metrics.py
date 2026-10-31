"""Recalculate all short stops, not only contiguous pauses lasting five seconds."""
import bisect
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import statistics

root = Path(__file__).resolve().parents[2]
old = root/'test_artifacts/visible_pursuit_repair_20261017'
closed = old/'closed_evidence/field_results/session_20261017_155029'
review = json.loads((old/'pursuit_closed_review.json').read_text(encoding='utf-8-sig'))
start = review['query_wall']
end = start + review['goal']['elapsed_s']
events = [json.loads(line) for line in (closed/'goal_001/events.jsonl').read_text(encoding='utf8').splitlines()]
statuses = sorted((v['stamp_ns']/1e9, v['status']) for v in events if v.get('kind') == 'plan_status')
native = []
observer = closed/'execution_observer/messages.jsonl'
with observer.open(encoding='utf8') as stream:
    for line in stream:
        row = json.loads(line)
        if row['topic'] != '/wildos/native_control_status':
            continue
        stamp = row['received_ros_ns']/1e9
        if start <= stamp <= end:
            data = json.loads(row['message']['data'])
            native.append((stamp, data['speed']))
native.sort()
seconds = {v: 0. for v in (.05, .2, .5, 1.)}
stopped_by_status = defaultdict(float)
covered = 0.
stamps = [v[0] for v in statuses]
for (stamp, speed), (next_stamp, _) in zip(native, native[1:]):
    dt = min(.2, max(0., next_stamp-stamp))
    covered += dt
    for threshold in seconds:
        if speed < threshold:
            seconds[threshold] += dt
    if speed < .05:
        i = bisect.bisect_right(stamps, stamp)-1
        stopped_by_status[statuses[i][1] if i >= 0 else 'before_first_status'] += dt
paths = json.loads((old/'execution_loop_review.json').read_text(encoding='utf8'))
lengths = [sum(math.dist(a[:2], b[:2]) for a, b in zip(v['points'], v['points'][1:]))
           for v in paths['dispatch']]
out = dict(scope='Closed observer receive-time weighting, left sample held until next sample; '
    'dt capped at 0.2s. Latest published status attribution is approximate, not internal execution state.',
    start_wall=start, end_wall=end, samples=len(native), covered_seconds=covered,
    below_speed_seconds={str(k): v for k, v in seconds.items()},
    stopped_seconds_by_latest_status=dict(sorted(stopped_by_status.items(), key=lambda v: -v[1])),
    dispatched_sections=dict(count=len(lengths), min_m=min(lengths), median_m=statistics.median(lengths),
        max_m=max(lengths), at_most_1m=sum(v <= 1. for v in lengths), at_most_2m=sum(v <= 2. for v in lengths)),
    observer_sha256=hashlib.file_digest(observer.open('rb'), 'sha256').hexdigest(),
    physical_retest=False)
Path(__file__).with_suffix('.json').write_text(json.dumps(out, indent=2), encoding='utf8')
print(json.dumps(out, indent=2))
