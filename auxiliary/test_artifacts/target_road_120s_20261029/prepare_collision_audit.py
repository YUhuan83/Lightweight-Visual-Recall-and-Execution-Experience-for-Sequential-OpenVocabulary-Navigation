"""Reuse existing closed-bag audits on v67; no runtime changes."""
import json
from pathlib import Path

parent = Path('/home/ly/execution_continuity_review_20261022')
old = parent / 'v58_confirmed_raw_handoff'
new = parent / 'v67_target_road_120s'
trial = new / 'full/trial_20261029_212751'
assert json.loads((trial / 'result.json').read_text())['closed']
events = [json.loads(line) for line in next(trial.glob('sessions/*/goal_001/events.jsonl')).read_text().splitlines()]
query = next(e['elapsed'] for e in events if e.get('kind') == 'query_sent')
stamps = [e['stamp_ns'] for e in events if e.get('status') == 'follow_failed'
          and 80 <= e['elapsed']-query <= 120]
for name in ('audit_tracking_collision.py', 'audit_collision_arc.py'):
    source = (old / name).read_text().replace(str(old / 'full/trial_20261028_094705'), str(trial))
    source = source.replace('(1793152166544698619,1793152203433633330)', repr(tuple(stamps)))
    source = source.replace('[1793152166544698619,1793152203433633330]', repr(stamps))
    (new / name).write_text(source)
print(stamps)
