"""Closed scalar control/dynamics evidence; does not diagnose contact from ACK."""
from pathlib import Path
import bisect,collections,json,math,statistics
OUT=Path(__file__).parent;ROOT=OUT.parents[1]
CLOSED=OUT.parent/'controller_source_field_20261018/closed_evidence'
SESSION=next((CLOSED/'field_results').glob('session_*'))
assert (SESSION/'session_result.json').is_file() and (SESSION/'goal_001/result.json').is_file()
events=[json.loads(x) for x in (SESSION/'goal_001/events.jsonl').read_text(encoding='utf8').splitlines() if x]
stalls=[x for x in events if x.get('reason')=='execution_stalled']
controls={};native=[];paths=[];rpp=[]
for line in (SESSION/'execution_observer/messages.jsonl').open(encoding='utf8'):
    row=json.loads(line);topic=row['topic'];t=row['received_ros_ns']/1e9;m=row['message']
    if topic=='/wildos/control_status':
        v=json.loads(m['data']);key=tuple((v.get('source_control') or {}).get(k) for k in ('sender_id','generation','sequence'))
        controls[key]=v
    elif topic=='/wildos/native_control_status':native.append((t,json.loads(m['data'])))
    elif topic=='/received_global_plan':paths.append((t,m))
    elif topic=='/wildos/cmd_vel_stamped':rpp.append((t,m))
nt=[t for t,_ in native];pt=[t for t,_ in paths]
reports=[]
for event in stalls:
    end=event['stamp_ns']/1e9;begin=end-5.
    samples=[(t,v) for t,v in native if begin-2<=t<=end+1.]
    raw=[]
    for t,v in samples:
        source=v.get('source_receipt') or {};key=tuple(source.get(k) for k in ('sender_id','generation','sequence'))
        c=controls.get(key,{})
        actual=v.get('actual') or {}
        raw.append(dict(wall=t,relative_to_failure=t-end,speed=v['speed'],position=v.get('position'),
            sent=v.get('sent'),actual=actual,source_applied=source.get('applied'),
            reason=c.get('reason'),target_speed=c.get('target_speed'),
            pending_gear=c.get('pending_gear'),current_gear=c.get('current_gear'),
            source_age=c.get('controller_source_age_ros_sec'),
            throttle_saturated=c.get('throttle_saturated'),telemetry_age=v.get('telemetry_age_sec'),
            wheel_contact=v.get('wheel_contact')))
    inner=[v for v in raw if -5<=v['relative_to_failure']<=0]
    def bounds(fn):
        values=[fn(v) for v in inner]
        values=[v for v in values if isinstance(v,(int,float)) and math.isfinite(v)]
        return dict(min=min(values),median=statistics.median(values),max=max(values)) if values else None
    positions=[v['position'] for v in inner]
    path=paths[bisect.bisect_right(pt,end)-1] if pt and pt[0]<=end else None
    report=dict(event=event,samples=len(inner),
        native_speed=bounds(lambda v:v['speed']),
        sent_throttle=bounds(lambda v:v['sent']['throttle']),sent_brake=bounds(lambda v:v['sent']['brake']),
        actual_throttle=bounds(lambda v:v['actual'].get('throttle')),
        actual_brake=bounds(lambda v:v['actual'].get('brake')),
        actual_wheelspeed=bounds(lambda v:v['actual'].get('wheelspeed')),
        rpm=bounds(lambda v:v['actual'].get('rpm')),
        reasons=dict(collections.Counter(v['reason'] for v in inner)),
        sent_gears=dict(collections.Counter(v['sent']['gear'] for v in inner)),
        actual_gears=dict(collections.Counter(v['actual'].get('gear') for v in inner)),
        native_displacement_m=math.dist(positions[0][:2],positions[-1][:2]) if positions else None,
        native_distance_m=sum(math.dist(a[:2],b[:2]) for a,b in zip(positions,positions[1:])),
        last_controller_path=path,scalar_rows=raw)
    reports.append(report)
(OUT/'stalls.json').write_text(json.dumps(reports,indent=2)+'\n',encoding='utf8')
print(json.dumps([{k:v for k,v in r.items() if k not in ('scalar_rows','last_controller_path')} for r in reports],indent=2))
