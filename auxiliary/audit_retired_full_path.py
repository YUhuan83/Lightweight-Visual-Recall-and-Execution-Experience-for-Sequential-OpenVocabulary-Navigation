"""Read only already-closed, independent controller progress audit and status logs."""
import hashlib,json
from pathlib import Path
repo=Path(__file__).parent.parent/'拟真';results=[]
for name,field in [('full_path_control_20261026','field'),('full_path_control_20261026','field_repeat'),('rolling_observation_20261026','field'),('observation_session_20261026','field'),('observation_heading_20261026','field'),('general_observation_20261026','field')]:
    root=repo/'test_artifacts'/name/field
    report=json.loads((root/'continuous_path_audit.json').read_text(encoding='utf8'))
    goal=next((root/'closed_core').glob('sessions/*/goal_001'))
    events=[json.loads(x) for x in (goal/'events.jsonl').read_text(encoding='utf8').splitlines()]
    q=next(e['elapsed'] for e in events if e['kind']=='query_sent')
    retires=[e for e in events if e.get('status') in ('exploration_finishing_segment','visible_target_handoff_finishing_segment','continuous_intent_handoff')]
    rows=[]
    for e in retires:
        at=e['elapsed']-q
        for p in report['path_identities']:
            if not p['first_elapsed']<=at<=p['last_elapsed']:continue
            changes=[t for t in p['controller_direction_transitions'] if at+.3<t<p['last_elapsed']]
            if not changes:continue
            rows.append(dict(event=e['status'],epoch=e.get('replay_inputs',{}).get('navigation_epoch'),
                elapsed=at,path_stamp_ns=p['path_stamp_ns'],path_last_elapsed=p['last_elapsed'],
                direction_changes_after_retirement=changes,remaining_at_retirement=e.get('remaining_distance_m'),
                owner=e.get('retired_exploration_id'),lease_distance_m=16.,lease_s=15.))
    results.append(dict(trial=report['trial'],rows=rows))
out=dict(scope='Closed plan events and independently audited actual controller path identities; event receipt vs bag receipt has subsecond transport skew. Changes within .3s excluded.',rounds=results)
path=repo/'test_artifacts/general_observation_20261026/retired_full_path_audit.json'
path.write_text(json.dumps(out,indent=2),encoding='utf8')
print(json.dumps(out,indent=2))
