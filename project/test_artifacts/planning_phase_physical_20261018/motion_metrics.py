from pathlib import Path
import json,bisect,collections
root=Path('/home/ly/planning_phase_physical_20261018')
t=sorted(root.glob('field_yaw_left90_search_*'))[-1]
assert (t/'result.json').exists(), 'driver active'
d=json.loads((t/'result.json').read_text());s=Path(d['session']);g=s/'goal_001'
assert (s/'session_result.json').exists() and (g/'result.json').exists(), 'recorders active'
start=next(c['wall'] for c in d['commands'] if c['text']=='green residential house')
end=start+json.loads((g/'result.json').read_text())['elapsed_s']
events=[json.loads(l) for l in (g/'events.jsonl').read_text().splitlines() if l]
states=sorted((e['stamp_ns']/1e9,e['status']) for e in events if e.get('kind')=='plan_status');ts=[v[0] for v in states]
native=[]
for l in (s/'execution_observer/messages.jsonl').read_text().splitlines():
 r=json.loads(l)
 if r['topic']=='/wildos/native_control_status' and start<=r['received_ros_ns']/1e9<=end:
  native.append((r['received_ros_ns']/1e9,json.loads(r['message']['data'])['speed']))
native.sort();seconds={t:0. for t in (.05,.2,.5,1.)};by=collections.defaultdict(float);covered=0.
for (t,v),(u,_) in zip(native,native[1:]):
 dt=min(.2,max(0.,u-t));covered+=dt
 for threshold in seconds:
  if v<threshold:seconds[threshold]+=dt
 if v<.05:
  i=bisect.bisect_right(ts,t)-1;by[states[i][1] if i>=0 else 'before_first_status']+=dt
out=dict(samples=len(native),covered_s=covered,below_speed_s={str(k):v for k,v in seconds.items()},stopped_s_by_latest_status=dict(by),scope='All passive Native receipt samples, left hold with dt capped .2s; latest public status attribution is approximate, not internal wait time')
(root/'motion_metrics.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
