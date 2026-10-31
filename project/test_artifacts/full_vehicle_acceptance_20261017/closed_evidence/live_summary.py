"""Bounded live status; JSONL only, never touches active SQLite recordings."""
from pathlib import Path
import collections,json,math,time
root=Path('/home/ly/full_vehicle_acceptance_20261017')
t=sorted(root.glob('field_yaw_left90_search_*'))[-1]
pid=int((t/'driver_child.pid').read_text())
sessions=[p.parent for p in (root/'field_results').glob('*/console_identity.json')
          if json.loads(p.read_text())['pid']==pid]
def rows(p):
    result=[]
    if p.exists():
        for line in p.read_text().splitlines():
            try:result.append(json.loads(line))
            except ValueError:pass
    return result
result=dict(trial=str(t),driver_closed=(t/'result.json').exists())
if sessions:
    s=sessions[0];g=s/'goal_001';events=rows(g/'events.jsonl');native=rows(g/'native_samples.jsonl')
    plans=[x for x in events if x.get('kind')=='plan_status']
    points=[x['position'] for x in native if isinstance(x.get('position'),list)]
    vision=rows(s/'vision_participation.jsonl')
    latest=next((x for x in reversed(vision) if x.get('kind')=='scores'),{})
    stationary=0.
    for row in reversed(native):
        if row.get('speed',1.)>=.05:break
        stationary=native[-1]['wall_time']-row['wall_time']
    result.update(session=str(s),elapsed_s=events[-1].get('elapsed') if events else None,
        distance_m=sum(math.dist(a[:2],b[:2]) for a,b in zip(points,points[1:])),
        current_stationary_s=stationary,last_status=plans[-1] if plans else None,
        status_counts=dict(collections.Counter(x.get('status') for x in plans)),
        score_calls=latest.get('state',{}).get('score_calls'),frontiers=latest.get('frontiers'),
        query=latest.get('query'),goal_closed=(g/'result.json').exists(),session_closed=(s/'session_result.json').exists())
    result['failures']=[x for x in plans if x.get('status') in ('follow_failed','no_vehicle_path','planning_recovery_waiting','recovery_failed')][-4:]
(root/'field/live_latest.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
