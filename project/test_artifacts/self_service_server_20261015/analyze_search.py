from collections import Counter
import json,math,time
from pathlib import Path
root=Path('/home/ly/wildos-console')
session=sorted((root/'results').glob('session_*'))[-1]
goal=session/'goal_001'
def rows(path):
    data=[]
    if path.exists():
        for line in path.read_text().splitlines():
            try:data.append(json.loads(line))
            except json.JSONDecodeError:pass
    return data
events=rows(goal/'events.jsonl');native=rows(goal/'native_samples.jsonl')
statuses=[e for e in events if e['kind']=='plan_status']
points=[v['position'] for v in native if isinstance(v.get('position'),list)]
distance=sum(math.dist(a[:2],b[:2]) for a,b in zip(points,points[1:]))
report=dict(session=str(session),read_only_snapshot=True,
    statuses=dict(Counter(e.get('status') for e in statuses)),events=dict(Counter(e['kind'] for e in events)),
    failures=[e for e in statuses if e.get('status') in ('follow_failed','no_vehicle_path','recovery_failed')][-4:],
    native_samples=len(native),travelled_native_m=distance,
    latest_native={k:native[-1].get(k) for k in ('speed','position','telemetry_age_sec','wall_time','actuator_ok','control_error')} if native else None,
    last_status=statuses[-1] if statuses else None)
print(json.dumps(report,indent=2))
if (goal/'result.json').exists():print((goal/'result.json').read_text()[:4000])
