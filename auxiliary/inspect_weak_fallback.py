import json,math
from pathlib import Path
repo=Path(__file__).parent.parent/'拟真';root=repo/'test_artifacts/observation_heading_20261026/field/closed_core'
goal=next(root.glob('sessions/*/goal_001'))
events=[json.loads(x) for x in (goal/'events.jsonl').read_text(encoding='utf8').splitlines()]
plans=[x for x in events if x.get('kind')=='plan_status']
for i,p in enumerate(plans):
    if p['status'] not in ('visible_target_local_fallback','visible_target_no_vehicle_path'):continue
    print(json.dumps(p,ensure_ascii=False))
    for q in plans[max(0,i-2):i+4]:
        print(json.dumps({k:q.get(k) for k in ('elapsed','status','reason','requested_goal_pose','bridge_odom_pose','background','goal')},ensure_ascii=False))
