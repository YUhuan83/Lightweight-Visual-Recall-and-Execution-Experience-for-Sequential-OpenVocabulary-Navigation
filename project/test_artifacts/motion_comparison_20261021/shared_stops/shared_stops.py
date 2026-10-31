"""Attribute near-stationary SDK intervals to paired public control/plan states."""
from compare import OUT,latest,q
import json,sqlite3,collections,re
from pathlib import Path
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
result=[]
for x in ('215010','224538','225946'):
 r=json.load(open(OUT/('trial_20261020_'+x+'.json')));zero=r['query_wall_start']
 c=sqlite3.connect('file:'+r['source_bag']['path']+'?mode=ro',uri=True);topics=dict(c.execute('select name,id from topics'))
 rows={}
 for key in ('native_control_status','control_status','plan_status'):
  out=[]
  for t,b in c.execute('select timestamp,data from messages where topic_id=? order by timestamp,id',(topics['/wildos/'+key],)):
   v=json.loads(deserialize_message(b,String).data);out.append(((v['wall_time'] if key=='native_control_status' else t/1e9)-zero,v))
  rows[key]=out
 n=rows['native_control_status'];control=rows['control_status'];plans=rows['plan_status'];ct=[t for t,v in control];pt=[t for t,v in plans]
 cross=collections.Counter();gear_episodes=[];run=None
 for (t,v),(tn,vn) in zip(n,n[1:]):
  dt=max(0.,min(tn,420.)-max(t,0.))
  if dt<=0:continue
  if not (v.get('telemetry_ok') is True and 0<=v.get('telemetry_age_sec',999)<=.5 and tn-t<=.5 and v['speed']<.1):continue
  cv=latest(control,ct,t);pv=latest(plans,pt,t)
  cross[(cv.get('reason'),pv.get('status'),pv.get('reason'))]+=dt
 for t,v in control:
  if not 0<=t<=420:continue
  if v.get('reason')=='gear_change' and run is None:run=t
  elif v.get('reason')!='gear_change' and run is not None:gear_episodes.append(t-run);run=None
 candidate_failures=collections.Counter();latencies=[];requests={}
 for line in (Path(r['session'])/'stack.log').open(errors='replace'):
  m=re.search(r'\[(\d+\.\d+)\]',line)
  if not m or not 0<=float(m.group(1))-zero<=420:continue
  req=re.search(r'ComputePath request sent: epoch=(\d+) candidate=(\d+) request=(\d+)',line)
  if req:requests[req.group(1),req.group(2)]=float(m.group(1))
  fail=re.search(r'ComputePath candidate failed: epoch=(\d+) candidate=(\d+) reason=(.*)',line)
  if fail:
   candidate_failures[fail.group(3).strip()]+=1
   begin=requests.get((fail.group(1),fail.group(2)))
   if begin is not None:latencies.append(float(m.group(1))-begin)
 stop=r['overall']['weights']['stationary_sec'];known=r['overall']['stationary_control_reasons_sec']
 result.append(dict(trial=x,source_bag_sha256=r['source_bag']['sha256'],stationary_sec=stop,
   stale_and_shift_fraction=(known.get('stale_command',0)+known.get('gear_change',0))/stop,
   stationary_control_reasons_sec=known,cross_tab=[dict(control=k[0],plan=k[1],plan_reason=k[2],seconds=v) for k,v in cross.most_common()],
   gear_change_episodes=len(gear_episodes),gear_change_episode_duration_sec=q(gear_episodes),
   candidate_failures=dict(candidate_failures),failed_candidate_request_to_failure_sec=q(latencies)))
 c.close()
(OUT/'shared_stop_mechanisms.json').write_text(json.dumps(dict(scope=__doc__,trials=result,complete_navigation_passed=False),indent=2)+'\n')
for r in result:print(json.dumps(r,indent=2))
