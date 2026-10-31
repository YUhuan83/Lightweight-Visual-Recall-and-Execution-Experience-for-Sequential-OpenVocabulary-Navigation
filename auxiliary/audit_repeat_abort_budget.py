"""Independent CLOSED log pairs: exhausted recovery budget retains owner."""
import bisect,hashlib,json,re
from pathlib import Path
repo=Path(__file__).parent.parent/'拟真';results=[]
for folder in ('confirmed_progress_repair_20261025','integrated_progress_20261025','full_path_control_20261026','observation_heading_20261026','general_observation_20261026','retirement_boundary_20261026'):
    base=repo/'test_artifacts'/folder
    goals=list(base.glob('field*/closed_core/sessions/*/goal_001'))+list(base.glob('field*/sessions/*/goal_001'))
    for goal in goals:
        p=goal/'events.jsonl';events=[json.loads(x) for x in p.read_text(encoding='utf8').splitlines()]
        stack=goal.parent/'stack.log'
        if not stack.exists():continue
        errors=[]
        for line in stack.read_text(encoding='utf8').splitlines():
            if 'bounded command arc' not in line:continue
            m=re.search(r'\[(\d+\.\d+)\]',line)
            if m:errors.append((float(m[1]),line))
        q=next(e['elapsed'] for e in events if e['kind']=='query_sent')
        rows=[]
        for i,e in enumerate(events):
            if not(e.get('status')=='follow_failed' and e.get('reason')=='follow_aborted' and e.get('action_status')==6 and e.get('retry_after_sec',0)>=8 and e.get('exploration_id')):continue
            at=e['stamp_ns']*1e-9;matched=[line for stamp,line in errors if 0<=at-stamp<1.5]
            if not matched:continue
            later=next((x for x in events[i+1:] if x.get('status')=='following' and x.get('goal') is not None),None)
            rows.append(dict(query_s=e['elapsed']-q,owner=e['exploration_id'],goal=e.get('exploration_goal'),
                retry_after_sec=e['retry_after_sec'],controller_error=matched[-1],
                next_following_same_owner=later is not None and later.get('exploration_id')==e['exploration_id'],
                next_following_query_s=later['elapsed']-q if later else None))
        if rows:results.append(dict(evidence=str(goal),events_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),rows=rows))
out=dict(scope='Fresh same-owner recovery budget exhausted, actual controller collision abort, later same-owner FollowPath. Not proof that every alternative frontier is reachable.',rounds=results)
root=repo/'test_artifacts/retirement_boundary_20261026'
(root/'repeated_abort_budget_audit.json').write_text(json.dumps(out,indent=2),encoding='utf8')
print(json.dumps(out,indent=2))
