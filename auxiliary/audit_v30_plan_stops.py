import json,math
from pathlib import Path
root=Path('/home/ly/execution_continuity_review_20261022/v30_confirmation_history')
a=json.loads((root/'field_result.json').read_text());trial=Path(a['trial']);assert a['result']['closed']
events=[json.loads(s) for s in next(trial.glob('sessions/*/goal_001/events.jsonl')).read_text().splitlines()]
rows=[]
for stop in sorted(a['fluency_summary.json']['stop_intervals'],key=lambda x:-x['duration'])[:4]:
    planned=[]
    for e in events:
        if e.get('kind')!='plan_status' or not stop['elapsed']-1<=e.get('elapsed',-1)<=stop['elapsed']+stop['duration']+1:continue
        if e['status']=='planning':
            sx,sy,h=e['bridge_odom_pose'];gx,gy,gh=e['requested_goal_pose']
            forward=(gx-sx)*math.cos(h)+(gy-sy)*math.sin(h)
            planned.append(dict(t=round(e['elapsed'],2),candidate=e['candidate_index'],start=[sx,sy,h],goal=[gx,gy,gh],forward_projection_m=round(forward,2)))
    rows.append(dict(stop=stop,planning=planned))
(root/'long_stop_candidate_audit.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows,indent=2))
