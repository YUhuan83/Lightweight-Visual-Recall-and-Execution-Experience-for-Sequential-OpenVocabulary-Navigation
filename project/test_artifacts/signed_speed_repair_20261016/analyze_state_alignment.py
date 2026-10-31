from pathlib import Path
import json,math,numpy as np
s=Path('/home/ly/wildos-console/audit_results/session_20261016_085025')
assert (s/'session_result.json').exists() and (s/'goal_001/result.json').exists()
control=[];native=[];ages=[]
for line in (s/'execution_observer/messages.jsonl').open():
 z=json.loads(line);topic=z['topic']
 if topic not in ('/wildos/control_status','/wildos/native_control_status'):continue
 d=json.loads(z['message']['data'])
 if topic=='/wildos/control_status':
  v=d.get('measured_longitudinal_speed')
  if d.get('enabled') and isinstance(v,(int,float)) and math.isfinite(v):
   control.append((d['stamp_ns']/1e9,float(v)))
   if d.get('state_source_age_ros_sec') is not None:ages.append(d['state_source_age_ros_sec'])
 else:
  v=d.get('velocity');f=d.get('direction')
  if v and f and math.hypot(*f[:2])>1e-6:
   value=(v[0]*f[0]+v[1]*f[1])/math.hypot(*f[:2]);t=d['wall_time']-float(d.get('telemetry_age_sec') or 0.)
   native.append((t,value))
a=np.array(control);b=np.array(sorted(native));a=a[(a[:,0]>b[0,0]+16)&(a[:,0]<b[-1,0]-16)]
results=[]
for lag in np.arange(-15.,15.001,.05):
 estimate=np.interp(a[:,0]+lag,b[:,0],b[:,1]);err=np.abs(a[:,1]-estimate)
 results.append(dict(native_time_offset_sec=round(float(lag),3),mae_mps=float(np.mean(err)),p95_abs_error_mps=float(np.percentile(err,95))))
results.sort(key=lambda x:x['mae_mps']);age_stats=dict(median=float(np.median(ages)),p95=float(np.percentile(ages,95)),max=float(max(ages)))
r=dict(scope='Scalar speed-series lag comparison using independent Native sample time approximation, not proof of every state field or exact acquisition timestamp; offsets shift Native comparison time relative to controller diagnostic time',samples=len(a),state_header_age_ros_sec=age_stats,best_offsets=results[:8],zero_offset=next(v for v in results if abs(v['native_time_offset_sec'])<.001),source_header_offset_comparison=min(results,key=lambda v:abs(v['native_time_offset_sec']+age_stats['median'])))
(s/'state_clock_alignment_review.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
