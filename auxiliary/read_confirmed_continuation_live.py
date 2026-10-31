"""Read text events only; never open a live recording."""
import collections,json,sys
from pathlib import Path
version=sys.argv[1] if len(sys.argv)>1 else 'v50_confirmed_continuation'
assert version in ('v50_confirmed_continuation','v51_confirmed_action_handoff','v53_confirmed_graph_destination')
root=Path('/home/ly/execution_continuity_review_20261022')/version
trial=max((root/'full').glob('trial_*'))
goals=list(trial.glob('sessions/*/goal_001'))
if not goals: print('starting'); raise SystemExit
events=[]
for line in (goals[0]/'events.jsonl').read_text().splitlines():
 try:events.append(json.loads(line))
 except json.JSONDecodeError:pass
query=next((e['elapsed'] for e in events if e.get('kind')=='query_sent'),0.)
selected=[e for e in events if e.get('status') in ('object_standoff_approach','confirmed_standoff_update_deferred','continuous_intent_handoff','confirmed_approach_failed','follow_failed','continuous_motion_failed','object_found','awaiting_visual_confirmation')]
print(json.dumps(dict(trial=str(trial),search_elapsed=events[-1]['elapsed']-query,counts=dict(collections.Counter(e.get('status') for e in events if e.get('status'))),recent=[dict(query_s=e['elapsed']-query,**{k:e[k] for k in ('status','goal','previous_goal','reason','retained_action','object_position','distance','proposed_goal') if k in e}) for e in ([e for e in selected if e.get('status')=='object_standoff_approach']+selected[-4:])]),indent=2))
