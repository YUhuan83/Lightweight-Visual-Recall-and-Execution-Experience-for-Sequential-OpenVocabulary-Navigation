"""Review only a fully closed trial; action event association is explicit."""
from pathlib import Path
import json, collections
root=Path('/home/ly/motion_handoff_field_20261018')
driver=sorted(root.glob('field_yaw_left90_search_*'))[-1]
assert (driver/'result.json').is_file(), 'driver active'
d=json.loads((driver/'result.json').read_text())
s=Path(d['session']);g=s/'goal_001'
assert (s/'session_result.json').is_file() and (g/'result.json').is_file(), 'recorders active'
events=[json.loads(line) for line in (g/'events.jsonl').read_text().splitlines() if line]
statuses=[e for e in events if e.get('kind')=='plan_status']
leases=[]
for i,e in enumerate(statuses):
    if e.get('status')!='exploration_finishing_segment':continue
    identity=e['retired_exploration_id']
    outcome=None
    for follow in statuses[i+1:]:
        if (follow.get('retired_exploration_id')==identity and follow.get('status') in (
                'exploration_segment_finished','exploration_handoff_stopped')):
            outcome=follow;break
        if (follow.get('status')=='following' and 'extension_rejected' not in follow
                and follow.get('exploration_id') and follow['exploration_id']!=identity):
            outcome=follow;break
        if follow.get('status')=='exploration_finishing_segment':break
    leases.append(dict(start=e, outcome=outcome,
        inferred_new_following=bool(outcome and outcome.get('status')=='following'),
        long_section=e['remaining_distance_m']>8.))
replans=[e for e in statuses if e.get('status')=='replanning_after_maneuver']
out=dict(session=str(s),candidate=json.loads((root/'manifest.json').read_text())['sha256'],
    handoff_leases=leases, long_borrowed_sections=sum(x['long_section'] for x in leases),
    handoff_stops=[e for e in statuses if e.get('status')=='exploration_handoff_stopped'],
    observed_new_following_after_retirement=sum(x['inferred_new_following'] for x in leases),
    replan_reasons=dict(collections.Counter(e.get('reason') for e in replans)),replans=replans,
    failures=[e for e in statuses if e.get('status') in ('follow_failed','no_vehicle_path','execution_unavailable')],
    scope='Closed public status evidence. New following is inferred from its accepted-action publication, '
          'not controller internals; no claim that unexercised failure branches were field-tested.')
(root/'closed_handoff_review.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(out,indent=2))
