from pathlib import Path
import json,collections,math,time,hashlib
r=Path('/home/ly/motion_handoff_field_20261018');trials=sorted(r.glob('field_scope_baseline_search_*'))
t=trials[-1]
def rows(p):
 if not p.exists():return []
 out=[]
 for line in p.read_text(errors='replace').splitlines():
  try:out.append(json.loads(line))
  except ValueError:pass
 return out
pid=int((t/'driver_child.pid').read_text());sessions=[p.parent for p in (r/'field_results').glob('*/console_identity.json') if json.loads(p.read_text())['pid']==pid]
out=dict(trial=str(t),console_tail=(t/'console.log').read_text(errors='replace')[-1800:])
if sessions:
 s=sessions[0];g=s/'goal_001';e=rows(g/'events.jsonl');n=rows(g/'native_samples.jsonl');p=[v['position'] for v in n if isinstance(v.get('position'),list)]
 a=rows(s/'vision_participation.jsonl');v=next((v for v in reversed(a) if v.get('kind')=='scores'),{})
 out.update(session=str(s),last_event=e[-1] if e else None,status_counts=dict(collections.Counter(x.get('status') for x in e if x.get('kind')=='plan_status')),
  native_distance_m=sum(math.dist(a[:2],b[:2]) for a,b in zip(p,p[1:])),latest_native=n[-1] if n else None,vision={k:x for k,x in v.items() if k!='candidates'},
  goal_closed=(g/'result.json').exists(),session_closed=(s/'session_result.json').exists())
 if out['goal_closed']:out['goal']=json.loads((g/'result.json').read_text())
 if out['session_closed']:out['session_result']=json.loads((s/'session_result.json').read_text())
print(json.dumps(out,indent=2))
