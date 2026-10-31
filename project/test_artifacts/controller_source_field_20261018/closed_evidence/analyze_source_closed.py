"""Source/receipt identity and latency audit from closed logs, no database reads."""
import collections,json,math,statistics
from pathlib import Path
root=Path('/home/ly/controller_source_field_20261018')
trial=sorted(root.glob('field_yaw_left90_search_*'))[-1]
assert (trial/'result.json').is_file(),'Driver active'
driver=json.loads((trial/'result.json').read_text())
session=Path(driver['session']);goal=session/'goal_001'
assert (session/'session_result.json').is_file() and (goal/'result.json').is_file(),'Recorders active'
def rows(p):
    with p.open() as f:
        for line in f:
            if line.strip():yield json.loads(line)
def key(frame):return tuple(frame[k] for k in ('sender_id','generation','sequence'))
def stats(values):
    values=sorted(values)
    return dict(count=len(values),min=values[0],median=statistics.median(values),
        p95=values[min(len(values)-1,int(len(values)*.95))],max=values[-1]) if values else None
issued={};native_observed=[];source_observer_count=0
for row in rows(session/'execution_observer/messages.jsonl'):
    if row['topic']=='/wildos/native_control_command':
        frame=json.loads(row['message']['data']);issued[key(frame)]=frame;source_observer_count+=1
        assert len(issued)<50000,'Unexpectedly large stream'
    elif row['topic']=='/wildos/native_control_status':native_observed.append(json.loads(row['message']['data']))
native=list(rows(goal/'native_samples.jsonl'))
events=list(rows(goal/'events.jsonl'));plans=[e for e in events if e.get('kind')=='plan_status']
ages=[];receipt_ages=[];delivery_ages=[];rpc=[];invalid=[];unknown=[];matched=0;claimed=0;late=[];rejections=collections.Counter()
for i,value in enumerate(native):
    ages.append(value['command_age'])
    if value.get('command_receipt_age') is not None:
        receipt_ages.append(value['command_receipt_age'])
        delivery_ages.append(value['command_age']-value['command_receipt_age'])
    if value.get('control_rpc_ms') is not None:rpc.append(value['control_rpc_ms'])
    if value.get('source_rejection'):rejections[value['source_rejection']]+=1
    receipt=value.get('source_receipt')
    if receipt and receipt.get('applied') is True:
        claimed+=1;frame=issued.get(key(receipt))
        if frame is None:unknown.append(i)
        elif (any(receipt[k]!=frame[k] for k in ('issued_wall_ns','expires_wall_ns','control'))
              or receipt['control']!=value['sent']
              or not receipt['issued_wall_ns']<=receipt['applied_wall_ns']<receipt['expires_wall_ns']):invalid.append(i)
        else:matched+=1
    elif value.get('control_acknowledged') is True and value['sent']['throttle']>0. and value['sent']['brake']<.9:
        late.append(dict(index=i,wall=value['wall_time'],source_receipt=receipt,command_age=value['command_age'],sent=value['sent']))
positions=[v['position'] for v in native if isinstance(v.get('position'),list)]
stops=[];start=None;last=None
for value in native:
    if value['speed']<.05:
        if start is None:start=value['wall_time']
        last=value['wall_time']
    else:
        if start is not None and last-start>=5.:stops.append(dict(start_wall=start,duration_sec=last-start))
        start=None
if start is not None and last-start>=5.:stops.append(dict(start_wall=start,duration_sec=last-start))
report=dict(driver=str(trial),session=str(session),goal=json.loads((goal/'result.json').read_text()),
    source_messages_observed=source_observer_count,native_goal_samples=len(native),
    source_age_sec=stats(ages),receipt_age_sec=stats(receipt_ages),
    source_age_at_callback_sec=stats(delivery_ages),control_rpc_ms=stats(rpc),
    source_claimed_valid=claimed,matched_source_receipts=matched,
    unknown_to_passive_observer=len(unknown),invalid_matching_receipts=invalid,
    acknowledged_positive_control_without_valid_source_receipt=late,
    rejection_native_sample_counts=dict(rejections),
    watchdog_samples=sum(bool(v.get('watchdog_stop')) for v in native),
    actuator_failed_samples=sum(v.get('actuator_ok') is not True for v in native),
    telemetry_unhealthy_samples=sum(v.get('telemetry_ok') is not True for v in native),
    distance_m=sum(math.dist(a[:2],b[:2]) for a,b in zip(positions,positions[1:])),
    max_departure_m=max((math.dist(p[:2],positions[0][:2]) for p in positions),default=None),
    stops_ge5s=stops,status_counts=dict(collections.Counter(v.get('status') for v in plans)),
    execution_unavailable_reasons=dict(collections.Counter(v.get('reason') for v in plans if v.get('status')=='execution_unavailable')),
    failed_states=[v for v in plans if v.get('status') in ('follow_failed','no_vehicle_path','execution_unavailable')],
    scope='Closed source/Native logs; source callback age derived from paired source/receipt monotonic anchors; not physical response latency')
(root/'source_control_review.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k not in ('failed_states','goal','acknowledged_positive_control_without_valid_source_receipt')},indent=2))
