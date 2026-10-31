import collections,json,re,sys
from pathlib import Path
root=Path(sys.argv[1]);g=next((root/'closed_core').glob('sessions/*/goal_001'));session=g.parent
events=[json.loads(x) for x in (g/'events.jsonl').read_text(encoding='utf8').splitlines()]
q=next(e['elapsed'] for e in events if e['kind']=='query_sent')
audit=json.loads((root/'displacement_audit_1.json').read_text(encoding='utf8'))
print('AUDIT_KEYS',list(audit))
for e in events:
    at=e['elapsed']-q
    if e.get('status') in ('exploration_retired','exploration_handoff_waiting','exploration_handoff_stopped','continuous_motion_failed','continuous_direction_retired','awaiting_control_release'):
        print(json.dumps({k:e[k] for k in ('status','reason','goal','exploration_id','retired_exploration_id','pending_exploration_id','boundary','remaining_distance_m') if k in e}|dict(query_s=round(at,3))))
vision=(session/'vision.log').read_text(encoding='utf8');query_time=next(e.get('stamp_ns') for e in events if e.get('stamp_ns'))*1e-9
lines=[]
for line in vision.splitlines():
    m=re.search(r'\[(\d+\.\d+)\]',line)
    if m and float(m[1])>=query_time:lines.append(line)
print('VISION',dict(callback=sum('Received callback' in x for x in lines),tf_found=sum('TF found for camera' in x for x in lines),tf_missing=sum('TF not found' in x for x in lines),no_detection=sum('No object detected' in x for x in lines)))
decisions=[json.loads(x) for x in (session/'decisions.jsonl').read_text(encoding='utf8').splitlines()]
scored=[e for e in decisions if e.get('kind')=='scored']
print('SCORED',dict(count=len(scored),eligible_min=min((x['eligible_headings'] for x in scored),default=None),eligible_max=max((x['eligible_headings'] for x in scored),default=None)))
