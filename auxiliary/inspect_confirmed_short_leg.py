from pathlib import Path
import json,collections
p=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts\localization_range_20261027\field');g=next((p/'closed_core').glob('sessions/*/goal_001'))
es=[json.loads(s) for s in (g/'events.jsonl').read_text(encoding='utf8').splitlines()];q=next(e['elapsed'] for e in es if e['kind']=='query_sent')
for e in es:
 t=e['elapsed']-q
 if e['kind']=='plan_status' and 123<t<143 and e.get('status') not in ('confirmation_before_standoff','confirmed_standoff_update_deferred'):
  print(round(t,3),{k:e[k] for k in ('status','goal','reason','maneuver_count','path_points','requested_goal_pose','requested_start_pose','background','query_generation','total_path_length_m','distance','path_length','error_code','controller_error','replay_inputs') if k in e})
log=(g.parent/'stack.log').read_text(encoding='utf8')
for l in log.splitlines():
 if any(x in l for x in ('Goal succeeded','goal succeeded','New path','Received a goal','cusp','Cusp','progress','Following','goal reached','Goal reached','terminate','Canceling','aborted','short')):print(l[:400])
