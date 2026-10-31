"""Measure acknowledged full-brake windows from closed Native telemetry."""
from pathlib import Path
import json,math
root=Path(__file__).parent
s=root/'closed_evidence/field_results/session_20261018_083258'
assert (s/'session_result.json').is_file()
samples=[]
for line in (s/'execution_observer/messages.jsonl').open(encoding='utf8'):
    row=json.loads(line)
    if row['topic']!='/wildos/native_control_status':continue
    d=json.loads(row['message']['data'])
    if not isinstance(d.get('position'),list) or not d.get('telemetry_ok'):continue
    samples.append(d)
windows=[];i=1
while i<len(samples):
    a=samples[i];old=samples[i-1]
    braking=lambda x:(x.get('control_acknowledged') is True and x.get('actuator_ok') is True
        and x.get('source_receipt',{}).get('applied') is True and x.get('sent',{}).get('brake',0)>=.9)
    if braking(a) and not braking(old) and old['speed']>=.25:
        group=[old,a];j=i+1
        while j<len(samples) and braking(samples[j]) and samples[j]['wall_time']-group[-1]['wall_time']<=.2:
            group.append(samples[j]);j+=1
            if group[-1]['speed']<=.05:break
        if group[-1]['speed']<=.05:
            distance=sum(math.dist(x['position'][:2],y['position'][:2]) for x,y in zip(group,group[1:]))
            speed=old['speed'];direction=sum(v*w for v,w in zip(old['velocity'][:2],old['direction'][:2]))
            bound=speed*.5+speed*speed/2.+.2
            windows.append(dict(start=old['wall_time'],initial_speed=speed,
                forward=direction>=0,distance_m=distance,elapsed_s=group[-1]['wall_time']-old['wall_time'],
                effective_deceleration=speed*speed/(2*distance) if distance>0 else None,
                predicted_bound_m=bound,bound_covers=distance<=bound,samples=len(group)))
        i=j
    else:i+=1
report=dict(scope='Closed on-site braking observations, not a universal tire/terrain guarantee. '
                  'Envelope candidate: v*0.5s + v^2/(2*1m/s^2) + 0.2m; starts at previous sample.',
    windows=windows,all_covered=all(x['bound_covers'] for x in windows))
(root/'braking_envelope_review.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf8')
print(json.dumps(report,indent=2))
