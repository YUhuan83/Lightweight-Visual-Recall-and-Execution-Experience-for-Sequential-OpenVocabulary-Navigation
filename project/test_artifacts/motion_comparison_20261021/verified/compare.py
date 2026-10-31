"""Compare closed physical trials on the same query-relative clock, without running simulation."""
from pathlib import Path
import json,sqlite3,math,bisect,collections,hashlib
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from nav_msgs.msg import Odometry,Path as RosPath
OUT=Path(__file__).resolve().parent
BASE=Path('/home/ly/graph_start_attachment_repair_20261020')
NEW=Path('/home/ly/confirmed_approach_recovery_20261020/v2')
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  while b:=f.read(4*1024*1024):h.update(b)
 return h.hexdigest()
def q(a):
 a=sorted(v for v in a if isinstance(v,(float,int)) and math.isfinite(v))
 return {'n':len(a),'p50':a[int((len(a)-1)*.5)],'p95':a[int((len(a)-1)*.95)],'p99':a[int((len(a)-1)*.99)],'max':a[-1]} if a else {}
def latest(rows,ts,t):
 i=bisect.bisect_right(ts,t)-1
 return rows[i][1] if i>=0 else {}
def strip(v):return {k:x for k,x in v.items() if k not in ('stamp_ns','timestamp','elapsed')}
def audit(trial):
 driver=json.loads((trial/'result.json').read_text()); assert driver['closed']
 session=Path(driver['session']); bag=session/'goal_001/interfaces/interfaces_0.db3'
 assert (bag.parent/'metadata.yaml').exists()
 c=sqlite3.connect('file:'+str(bag)+'?mode=ro',uri=True)
 topics={n:(i,t) for i,n,t in c.execute('select id,name,type from topics')}
 def raw(topic):
  if topic not in topics:return []
  return c.execute('select timestamp,data from messages where topic_id=? order by timestamp,id',(topics[topic][0],)).fetchall()
 queries=raw('/spot1/object_search/query'); assert queries
 zero=queries[0][0]/1e9
 js={}
 for topic in ('/wildos/native_control_status','/wildos/control_status','/wildos/plan_status','/wildos/execution_feedback','/spot1/object_confirmation','/spot1/object_localization','/spot1/visible_target_approach','/spot1/graphnav_planner/route_status','/wildos/memory_selection','/wildos/native_control_command','/spot1/graphnav_planner/retired_exploration'):
  rows=[]
  for t,b in raw(topic):
   v=json.loads(deserialize_message(b,String).data)
   w=v['wall_time'] if topic=='/wildos/native_control_status' else t/1e9
   rows.append((w-zero,v))
  js[topic]=rows
 n=js['/wildos/native_control_status'];control=js['/wildos/control_status'];plans=js['/wildos/plan_status']
 nt=[t for t,v in n];ct=[t for t,v in control];pt=[t for t,v in plans]
 end=min(420.,n[-1][0]); origin=n[max(0,bisect.bisect_left(nt,0)-1)][1]['position'][:2]
 def aggregate(lo,hi):
  w=collections.Counter();cr=collections.Counter();pr=collections.Counter()
  for (t,v),(tn,vn) in zip(n,n[1:]):
   dt=max(0,min(tn,hi)-max(t,lo))
   if dt<=0:continue
   valid=v.get('telemetry_ok') is True and v.get('position') is not None and isinstance(v.get('telemetry_age_sec'),(float,int)) and 0<=v['telemetry_age_sec']<=.5 and tn-t<=.5
   w['observed_sec' if valid else 'unobserved_sec']+=dt
   if not valid:continue
   stopped=v['speed']<.1
   w['stationary_sec' if stopped else 'moving_sec']+=dt
   if stopped:
    if (v.get('actual') or {}).get('brake',0)>=.5:w['stationary_actual_brake_sec']+=dt
    if (v.get('actual') or {}).get('throttle',0)>.05:w['stationary_actual_throttle_sec']+=dt
    cv=latest(control,ct,t);pv=latest(plans,pt,t)
    cr[cv.get('reason','missing')]+=dt;pr[str(pv.get('status'))+':'+str(pv.get('reason'))]+=dt
   if v.get('watchdog_stop'):w['watchdog_sec']+=dt
  edge_missing=max(0.,min(hi,n[0][0])-lo)+max(0.,hi-max(lo,n[-1][0]))
  w['unobserved_sec']+=edge_missing
  w['unobserved_native_edges_sec']=edge_missing
  sel=[(t,v) for t,v in n if lo<=t<=hi]
  if not sel:return {}
  last=sel[-1][1]['position'][:2];first=sel[0][1]['position'][:2]
  return dict(range_sec=[lo,hi],weights=dict(w),stationary_percent=100*w['stationary_sec']/w['observed_sec'] if w['observed_sec'] else None,
   net_m=math.dist(first,last),distance_from_query_start_m=math.dist(origin,last),max_distance_from_query_start_m=max(math.dist(origin,v['position'][:2]) for t,v in sel),
   first_xy=first,last_xy=last,stationary_control_reasons_sec=dict(cr),stationary_plan_states_sec=dict(pr))
 od=[]
 for t,b in raw('/spot1/odom'):
  m=deserialize_message(b,Odometry);p=m.pose.pose.position;od.append((t/1e9-zero,[p.x,p.y]))
 planned=[];replans=[];episodes=[];src_active=None;last_state=None;timeline=[]
 for t,v in plans:
  if not 0<=t<=end:continue
  state=(v.get('status'),v.get('reason'))
  if state==last_state:continue
  last_state=state
  nv=latest(n,nt,t);cv=latest(control,ct,t)
  if state[0] in ('planned','following','planning','replanning_after_maneuver','maneuver_done','execution_unavailable','execution_recovered','segment_done','route_done','route_recovery_exhausted','follow_failed','no_vehicle_path'):
   timeline.append(dict(t=t,plan=v,position=nv.get('position'),speed=nv.get('speed'),control_reason=cv.get('reason')))
  if state[0]=='planned':planned.append(dict(t=t,**v))
  if state[0]=='replanning_after_maneuver':replans.append(dict(t=t,**v))
  if state[0]=='execution_unavailable' and src_active is None:src_active=dict(start_sec=t,first_reason=state[1])
  elif state[0]=='execution_recovered' and src_active is not None:
   src_active['end_sec']=t;episodes.append(src_active);src_active=None
 if src_active:episodes.append(dict(**src_active,end_sec=None))
 commands=js['/wildos/native_control_command'];firsts={}
 for t,v in n:
  rec=v.get('source_receipt')
  if rec and rec.get('sequence') not in firsts:firsts[rec['sequence']]=(t,v)
 timings=[]
 for t,v in commands:
  if not 0<=t<=end:continue
  native=firsts.get(v['sequence']); issued=v['issued_wall_ns']/1e9
  timings.append(dict(t=t,sequence=v['sequence'],issued_wall_ns=v['issued_wall_ns'],recorder_delay_ms=(zero+t-issued)*1000,
    first_native_seen_ms=((zero+native[0]-issued)*1000 if native else None),
    native_callback_delay_ms=((zero+native[0]-native[1]['command_receipt_age']-issued)*1000 if native and native[1].get('command_receipt_age') is not None else None)))
 def times_for(topic):
  return [t for t,b in raw(topic) if t/1e9>=zero]
 masks=times_for('/spot1/object_mask')
 received=[]
 for t,b in raw('/wildos/planned_path'):
  if not 0<=t/1e9-zero<=end:continue
  m=deserialize_message(b,RosPath)
  received.append(dict(t=t/1e9-zero,points=len(m.poses),length_m=sum(math.hypot(a.pose.position.x-b.pose.position.x,a.pose.position.y-b.pose.position.y) for a,b in zip(m.poses,m.poses[1:])),start=([m.poses[0].pose.position.x,m.poses[0].pose.position.y] if m.poses else None),end=([m.poses[-1].pose.position.x,m.poses[-1].pose.position.y] if m.poses else None)))
 config=json.loads((trial/'config.json').read_text())
 scene=json.loads((session/'scene.json').read_text());runtime=json.loads((trial/'runtime_identity.json').read_text())
 verified=json.loads((session/'verified_sources.json').read_text())
 report=dict(scope=__doc__,trial=str(trial),session=str(session),query_wall_start=zero,query=deserialize_message(queries[0][1],String).data,closed=True,video=driver.get('video'),
  source_bag=dict(path=str(bag),bytes=bag.stat().st_size,sha256=sha(bag)),overall=aggregate(0,end),first_90sec=aggregate(0,min(90,end)),minutes=[aggregate(t,min(t+60,end)) for t in range(0,math.ceil(end),60)],
  native_samples=len(n),rpc_ms=q([v.get('control_rpc_ms') for t,v in n if 0<=t<=end]),poll_ms=q([v.get('poll_roundtrip_ms') for t,v in n if 0<=t<=end]),
  telemetry_age_ms=q([v.get('telemetry_age_sec',999)*1000 for t,v in n if 0<=t<=end]),
  actuator_bad_samples=sum(v.get('actuator_ok') is not True for t,v in n if 0<=t<=end),lease_tripped_samples=sum(bool((v.get('actual') or {}).get('wildosControlLeaseTripped')) for t,v in n if 0<=t<=end),
  source_timing={k:q([x.get(k) for x in timings]) for k in ('recorder_delay_ms','first_native_seen_ms','native_callback_delay_ms')},
  command_publish_gap_ms=q([(b['issued_wall_ns']-a['issued_wall_ns'])/1e6 for a,b in zip(timings,timings[1:])]),
  source_unavailable_episodes=episodes,planned=planned,replans=replans,motion_paths=received,
  first_mask_sec=(masks[0]/1e9-zero if masks else None),mask_messages=len(masks),
  first_localization_sec=next((t for t,v in js['/spot1/object_localization'] if t>=0),None),
  first_confirmation_sec=next((t for t,v in js['/spot1/object_confirmation'] if t>=0),None),
  first_visible_hint_sec=next((t for t,v in js['/spot1/visible_target_approach'] if t>=0),None),
  plan_status_counts=dict(collections.Counter(v.get('status') for t,v in plans if 0<=t<=end)),
  execution_reasons=dict(collections.Counter(v.get('reason') for t,v in js['/wildos/execution_feedback'] if 0<=t<=end)),
  scene=scene,runtime_identity=runtime,verified_sources=verified,config=config,complete_navigation_passed=False,
  stationary_native_intervals=[],limits='Read-only closed public interfaces; same source does not imply identical sampled maps or scheduling. A mask, path or distance alone is not visual success.')
 run=None
 for (t,v),(tn,vn) in zip(n,n[1:]):
  if t<0 or t>end:continue
  good=v.get('telemetry_ok') is True and 0<=v.get('telemetry_age_sec',999)<=.5 and tn-t<=.5 and v['speed']<.1
  if good and run is None:run=t
  if not good and run is not None:
   if t-run>=5:report['stationary_native_intervals'].append(aggregate(run,t))
   run=None
 if run is not None and end-run>=5:report['stationary_native_intervals'].append(aggregate(run,end))
 (OUT/(trial.name+'.json')).write_text(json.dumps(report,indent=2)+'\n')
 (OUT/(trial.name+'_timeline.json')).write_text(json.dumps(timeline,indent=2)+'\n')
 (OUT/(trial.name+'_source_timing.json')).write_text(json.dumps(timings,indent=2)+'\n')
 c.close()
 print(json.dumps(dict(trial=trial.name,overall=report['overall'],rpc=report['rpc_ms'],source=report['source_timing'],first_plan=planned[0] if planned else None,first_replan=replans[0] if replans else None,unavailable_episodes=episodes,mask=report['first_mask_sec']),indent=2),flush=True)
 return report
if __name__=='__main__':
 reports=[audit(p) for p in [BASE/'full/trial_20261020_215010',NEW/'full/trial_20261020_224538',NEW/'full/trial_20261020_225946']]
 a,b=reports[0],reports[1]
 common=a['verified_sources'].keys()&b['verified_sources'].keys()
 identity=dict(production_diff=[k for k in common if a['verified_sources'][k]!=b['verified_sources'][k]],
   new_trials_sources_equal=reports[1]['verified_sources']==reports[2]['verified_sources'],
   frozen_manifests=[str(BASE/'frozen_source_manifest.json'),str(NEW/'frozen_source_manifest.json')])
 (OUT/'identity_comparison.json').write_text(json.dumps(identity,indent=2)+'\n'); print('IDENTITY',json.dumps(identity))
