import bisect
import json
from pathlib import Path

root=Path(__file__).parent.parent/'拟真/test_artifacts/command_continuity_regression_20261025'
report={}
for version in ('v28','v30','v28_recheck'):
    session=next((root/version/'sessions').glob('session_*'))
    goal=session/'goal_001'
    native=[json.loads(l) for l in (goal/'native_samples.jsonl').read_text().splitlines()]
    events=[json.loads(l) for l in (goal/'events.jsonl').read_text().splitlines()]
    plans=[e for e in events if e['kind']=='plan_status']
    origin=next(e['stamp_ns']*1e-9-e['elapsed'] for e in plans)
    windows=[]
    for start,end in ((0.,90.),(90.,180.)):
        covered=low=0.
        for a,b in zip(native,native[1:]):
            t=a['wall_time'];dt=b['wall_time']-t
            if not 0<dt<=.5:continue
            overlap=max(0.,min(t+dt,origin+end)-max(t,origin+start))
            covered+=overlap
            if abs(a['speed'])<.1:low+=overlap
        windows.append(dict(start=start,end=end,coverage=covered,near_stop_s=low,near_stop_pct=100*low/covered))
    stops=json.loads((root/version/'fluency_summary.json').read_text())['stop_intervals']
    details=[]
    for stop in stops:
        if stop['duration']<4:continue
        st=stop['elapsed'];en=st+stop['duration']
        timeline=[{k:e[k] for k in ('elapsed','status','reason','background','candidate_index','extension_rejected') if k in e}
                  for e in plans if st-3<=e['elapsed']<=en+.5]
        details.append(dict(stop=stop,events=timeline))
    report[version]=dict(windows=windows,long_stops=details)
(root/'stop_windows.json').write_text(json.dumps(report,indent=2),encoding='utf8')
print(json.dumps({v:d['windows'] for v,d in report.items()},indent=2))
