"""Count every planner attempt, including failures hidden by fallback/comparison."""
from pathlib import Path
from collections import Counter
import json
import re
import statistics

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).parent
CLOSED = ROOT/'test_artifacts/full_vehicle_acceptance_20261017/closed_evidence'
report = json.loads((CLOSED/'pursuit_closed_review.json').read_text(encoding='utf8'))
start = report['query_wall']
end = start + report['goal']['elapsed_s']
session = CLOSED/'field_results/session_20261017_172053'
pending, attempts, failures = {}, [], []
for line in (session/'stack.log').read_text(encoding='utf8').splitlines():
    stamp = re.search(r'\[(\d+\.\d+)\]', line)
    if not stamp or not start <= float(stamp[1]) <= end:
        continue
    t = float(stamp[1])
    sent = re.search(r'ComputePath request sent: epoch=(\d+) candidate=(\d+) request=(\d+)', line)
    result = re.search(r'ComputePath result: epoch=(\d+) candidate=(\d+) status=(\d+) poses=(\d+)', line)
    failed = re.search(r'ComputePath candidate failed: epoch=(\d+) candidate=(\d+) reason=(.*)', line)
    if sent:
        epoch, candidate, serial = map(int, sent.groups())
        key = epoch, candidate
        if key in pending:
            attempts.append(dict(**pending.pop(key), result='superseded_or_cancelled'))
        pending[key] = dict(epoch=epoch, candidate=candidate, serial=serial, sent=t-start)
    elif result:
        epoch, candidate, state, poses = map(int, result.groups())
        key = epoch, candidate
        if key in pending:
            item = pending.pop(key)
            attempts.append(dict(**item, elapsed_s=t-start-item['sent'], action_status=state,
                                 poses=poses, result='success' if state == 4 and poses else 'failed'))
    if failed:
        failures.append(dict(t=t-start, epoch=int(failed[1]), candidate=int(failed[2]), reason=failed[3]))
attempts.extend(dict(**x, result='no_current_result_logged') for x in pending.values())
durations = [x['elapsed_s'] for x in attempts if 'elapsed_s' in x]
output = dict(scope='Closed query window. Results matched by logged epoch/candidate; '
                   'unmatched/cancelled requests remain explicit. Not CPU time or private-map lock attribution.',
    result_counts=dict(Counter(x['result'] for x in attempts)),
    candidate_failure_reasons=dict(Counter(x['reason'] for x in failures)),
    roundtrip_seconds=dict(count=len(durations), median=statistics.median(durations),
        p95=sorted(durations)[int(.95*(len(durations)-1))], max=max(durations)),
    attempts=attempts, failures=failures)
(OUT/'planner_timing.json').write_text(json.dumps(output, indent=2), encoding='utf8')
print(json.dumps({k: v for k, v in output.items() if k not in ('attempts', 'failures')}, indent=2))
