from pathlib import Path
import json,collections
s=Path('/home/ly/wildos-console/audit_results/session_20261016_145143/goal_001')
def lines(name):return [json.loads(v) for v in (s/name).read_text().splitlines() if v]
n=lines('native_samples.jsonl');e=lines('events.jsonl')
moving=[v for v in n if v.get('speed',0.)>.1]
fail=[v for v in e if v.get('status')=='no_vehicle_path']
print(json.dumps(dict(last_moving=[dict(wall_time=v['wall_time'],speed=v['speed'],
    longitudinal=sum(a*b for a,b in zip(v['velocity'][:2],v['direction'][:2])),position=v['position']) for v in moving[-4:]],
    no_path=[dict(stamp=v['stamp_ns'],id=v.get('exploration_id'),reasons=v.get('reasons')) for v in fail],
    recovery=[v for v in e if v.get('status','').startswith('recovery')]),indent=2))
