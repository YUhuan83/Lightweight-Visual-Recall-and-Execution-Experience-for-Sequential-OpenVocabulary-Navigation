import json,re
from pathlib import Path
root=Path('/home/ly/execution_continuity_review_20261022/v51_confirmed_action_handoff')
r=json.loads((root/'field_result.json').read_text());assert r['result']['closed'] and not r['postcheck']['processes']
g=next(Path(r['trial']).glob('sessions/*/goal_001'))
ev=[json.loads(l) for l in (g/'events.jsonl').read_text().splitlines()]
q=next(e['elapsed'] for e in ev if e.get('kind')=='query_sent')
offset=next(e['stamp_ns']*1e-9-(e['elapsed']-q) for e in ev if e.get('stamp_ns'))
for e in ev:
 t=e['elapsed']-q
 if 60<=t<=160 and e.get('status') and e['status']!='confirmed_standoff_update_deferred':
  print(json.dumps(dict(query_s=t,**{k:v for k,v in e.items() if k in ('status','reason','goal','previous_goal','graph_goal','retained_action','path_stamp_ns','path_source_stamp_ns','failure_reasons','candidate_goals','replay_inputs','actual_start_pose','motion_sections_remaining')})))
for l in (g.parent/'stack.log').read_text().splitlines():
 stamps=re.findall(r'\[([0-9]{10}\.[0-9]+)\]',l)
 if not stamps: continue
 t=float(stamps[0])-offset
 if 60<=t<=160 and any(s in l.lower() for s in ('warn','error','failed','abort','path found','make progress','exceeded')):print('STACK',round(t,3),l[:500])
