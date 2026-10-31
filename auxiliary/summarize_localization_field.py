import json,collections
from pathlib import Path
r=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts\localization_range_20261027\field');g=next((r/'closed_core').glob('sessions/*/goal_001'));es=[json.loads(s) for s in (g/'events.jsonl').read_text(encoding='utf8').splitlines()];q=next(e['elapsed'] for e in es if e['kind']=='query_sent')
for e in es:
 if e['kind']=='plan_status' and e.get('status') in ('confirmation_before_standoff','confirmed_standoff_update_deferred','target_evidence_rejected','awaiting_visual_confirmation','object_standoff_approach','target_instance_released','unconfirmed_target_release_pending'):
  if e['status'] in ('confirmation_before_standoff','confirmed_standoff_update_deferred') and e['elapsed']<q+125:continue
  print(round(e['elapsed']-q,2),{k:e[k] for k in ('status','goal','proposed_goal','object_position','reason','distance','vehicle_position','instance_id') if k in e})
t=json.loads((r/'range_transport_audit.json').read_text(encoding='utf8'));t.pop('localized_observer_ranges_m',None);print('TRANSPORT',json.dumps(t))
a=json.loads((r/'closed_arrival_audit.json').read_text(encoding='utf8'));print('ARRIVAL',{k:v for k,v in a.items() if k not in ('confirmations','controller_abort_lines')})
