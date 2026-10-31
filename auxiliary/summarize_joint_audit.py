import json,sys
from pathlib import Path
base=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts')
for version in ('rolling_observation_20261026','observation_session_20261026'):
    root=base/version/'field'
    d=json.loads((root/'displacement_audit.json').read_text(encoding='utf8'))
    goal=next((root/'closed_core').glob('sessions/*/goal_001'))
    samples=[json.loads(l) for l in (goal/'native_samples.jsonl').read_text(encoding='utf8').splitlines()]
    print(version)
    for s in d['long_stops'][:3]:
        print(json.dumps({k:v for k,v in s.items() if k!='events'},ensure_ascii=False))
        for e in s['events']:print(json.dumps(e,ensure_ascii=False))
        # Original same-frame control versus actual velocity, no inferred state.
        origin=next(x for x in samples if x['speed']<.1 and abs(x['position'][0]-s['position'][0])<.001 and abs(x['position'][1]-s['position'][1])<.001)['wall_time']
        subset=[x for x in samples if origin<=x['wall_time']<origin+s['duration_s']]
        for x in subset[::max(1,len(subset)//3)]:print(json.dumps({k:x.get(k) for k in ('wall_time','speed','telemetry_age_sec','command_age','source_rejection','sent','watchdog_stop')},ensure_ascii=False))
