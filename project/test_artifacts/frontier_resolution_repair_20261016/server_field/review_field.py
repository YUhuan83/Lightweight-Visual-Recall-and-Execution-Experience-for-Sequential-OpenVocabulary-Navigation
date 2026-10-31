"""Closed-recording review; no large navigation graph deserialization."""
from pathlib import Path
import collections,json,math,sqlite3
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

root=Path('/home/ly/frontier_resolution_repair_20261016')
field=root/'field'
report=json.loads((field/'closed_review.json').read_text())
s=Path(report['session']);g=s/'goal_001';trial=Path(report['trial']['session']).parent
# The exercised closed analyzer already enforces all three completion barriers.
assert (s/'session_result.json').is_file() and (g/'result.json').is_file()
driver=sorted(root.glob('field_scope_baseline_search_*'))[-1]
assert (driver/'result.json').is_file()
events=[json.loads(x) for x in (g/'events.jsonl').read_text().splitlines() if x]
plans=[x for x in events if x.get('kind')=='plan_status']
with sqlite3.connect(next((g/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True) as connection:
    def records(topic):
        pair=connection.execute('select id,type from topics where name=?',(topic,)).fetchone()
        if not pair:return []
        cls=get_message(pair[1])
        return [(ts,deserialize_message(data,cls)) for ts,data in connection.execute(
            'select timestamp,data from messages where topic_id=? order by timestamp',(pair[0],))]
    routes=[dict(received_ns=ts,**json.loads(message.data)) for ts,message in records('/spot1/graphnav_planner/route_status')]
    choices=records('/spot1/graphnav_planner/vehicle_choice')
    selections=records('/spot1/graphnav_planner/selection_ack')
    masks=len(records('/spot1/object_mask'))
    localizations=len(records('/spot1/object_localization'))
    confirmations=len(records('/spot1/object_confirmation'))
    recalls=records('/spot1/remembered_object_candidates')
    gaps=[dict(start_ns=a['received_ns'],end_ns=b['received_ns'],seconds=(b['received_ns']-a['received_ns'])/1e9)
          for a,b in zip(routes,routes[1:]) if b['received_ns']-a['received_ns']>5e9]
    failure_counts=collections.Counter((x.get('reason'),x.get('detail')) for x in plans
        if x.get('status') in ('no_vehicle_path','follow_failed','recovery_failed'))
    stop_details=[]
    for stop in report['stationary_intervals_ge5s']:
        start=stop['start_wall'];end=start+stop['duration_sec']
        preceding=next((x for x in reversed(plans) if x['stamp_ns']/1e9<=start),None)
        changes=[x for x in plans if start<=x['stamp_ns']/1e9<=end]
        stop_details.append(dict(**stop,preceding=preceding,status_changes=changes))
    summary=dict(session=str(s),candidate='274c3ef2b5cecd1522579ccd4eea2b5a3c4bab649f5e1669c9d79226de7079a6',
        memory_priors_disabled=True,scope='Common current adapter with three new memory priors disabled; not untouched upstream WildOS',
        goal=report['goal'],native_distance_m=report['native_distance_m'],max_departure_m=report['max_departure_m'],
        final_departure_m=report['final_departure_m'],status_counts=report['status_counts'],
        stationary_intervals_ge5s=report['stationary_intervals_ge5s'],
        longest_stationary_s=max((x['duration_sec'] for x in report['stationary_intervals_ge5s']),default=0),
        route_decisions=len(routes),complete_route_decisions=sum(bool(x.get('complete')) for x in routes),
        route_gaps_ge5s=gaps,retirement_reasons=report['retirement_reasons'],
        observed_route_ack_messages=report['observed_route_ack_messages'],goal_drift_within_id=report['goal_drift_within_id'],
        vehicle_choices=len(choices),selection_ack_messages=len(selections),masks=masks,localizations=localizations,
        confirmations=confirmations,recall_messages=len(recalls),
        failure_counts=[dict(reason=k[0],detail=k[1],count=v) for k,v in failure_counts.items()],
        first_failures=[x for x in plans if x.get('status') in ('no_vehicle_path','follow_failed','planning_recovery_waiting','recovery_started','recovery_failed')][:12],
        session_errors=report['session_result'].get('errors'),vision_final=report['vision_final'])
(field/'baseline_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
(field/'stationary_detail.json').write_text(json.dumps(stop_details,indent=2)+'\n')
print(json.dumps(summary,indent=2))
