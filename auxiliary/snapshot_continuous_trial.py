"""Read text progress only; never inspect a recording SQLite database."""
import collections,json,math,time
from pathlib import Path
root=Path('/home/ly/execution_continuity_review_20261022')
trials=list((root/'v23_arc_heading/full').glob('trial_*'))+list((root/'v24_continuous_exploration/full').glob('trial_*'))
trial=max(trials,key=lambda p:p.name)
def lines(path):
    rows=[]
    if path.exists():
        for line in path.read_text().splitlines():
            try:rows.append(json.loads(line))
            except json.JSONDecodeError:pass
    return rows
goals=list(trial.glob('sessions/*/goal_001'));result=trial/'result.json'
out=dict(trial=str(trial),closed_result=json.loads(result.read_text()) if result.exists() else None)
if goals:
    events=lines(goals[0]/'events.jsonl');native=lines(goals[0]/'native_samples.jsonl');plans=[e for e in events if e['kind']=='plan_status']
    out.update(latest_plan=plans[-1] if plans else None,states=dict(collections.Counter(e['status'] for e in plans)))
    if native:out.update(native_count=len(native),last_native_age=time.time()-native[-1]['wall_time'],last_speed=native[-1]['speed'],net_displacement_m=math.dist(native[0]['position'][:2],native[-1]['position'][:2]))
print(json.dumps(out))
