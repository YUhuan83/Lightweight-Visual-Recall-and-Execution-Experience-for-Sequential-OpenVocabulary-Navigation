import json
from pathlib import Path
root=Path('/home/ly/execution_continuity_review_20261022/v27_fresh_bearing')
trial=root/'full/trial_20261025_154400'
assert json.loads((trial/'result.json').read_text())['closed']
goal=next(trial.glob('sessions/*/goal_001'))
events=[json.loads(s) for s in (goal/'events.jsonl').read_text().splitlines()]
rows=[]
for e in events:
    if e.get('kind')=='plan_status' and 126<=e.get('elapsed',-1)<=156:
        rows.append({k:v for k,v in e.items() if k not in ('schema_version','phase','query','kind','replay_inputs')})
out=dict(events=rows)
gaps=json.loads((trial/'command_gaps.json').read_text())
out['gaps']=gaps
(root/'long_gap_audit.json').write_text(json.dumps(out,indent=2))
print(json.dumps(rows,indent=2))
