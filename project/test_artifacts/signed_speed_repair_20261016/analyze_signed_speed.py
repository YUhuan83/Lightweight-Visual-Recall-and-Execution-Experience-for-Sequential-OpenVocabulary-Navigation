from pathlib import Path
import json,math,collections
s=Path('/home/ly/wildos-console/audit_results/session_20261016_085025')
assert (s/'session_result.json').exists() and (s/'goal_001/result.json').exists()
counts=collections.Counter();violations=[];examples=[];signed_count=0;total=0
for line in (s/'execution_observer/messages.jsonl').open():
 z=json.loads(line)
 if z['topic']!='/wildos/control_status':continue
 value=z['message'];d=json.loads(value['data']);total+=1;counts[d['reason']]+=1
 v=d.get('measured_longitudinal_speed');target=d.get('target_speed',0.)
 if isinstance(v,(float,int)) and math.isfinite(v):
  signed_count+=1
  if d['reason']=='tracking' and abs(target)>.01 and v*math.copysign(1.,target)<-.150001:violations.append(dict(kind='opposite_motion_not_braked',value=d))
 if d['reason']=='opposite_motion_braking':
  if d['throttle']!=0 or d['brake']<.999:violations.append(dict(kind='bad_braking_outputs',value=d))
  if len(examples)<8:examples.append(d)
r=dict(scope='Recorded controller diagnostics; full observed same-message direction/brake contract, not all physics or task acceptance',samples=total,finite_signed_samples=signed_count,reason_counts=dict(counts),contract_violations=violations,opposite_braking_examples=examples)
(s/'signed_speed_review.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
