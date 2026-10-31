from pathlib import Path
import json,collections,math,sqlite3,hashlib,zipfile,time
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.motion_path import motion_sections
r=Path('/home/ly/motion_handoff_field_20261018');p=Path('/home/ly/motion_handoff_field_20261018/field')
t=sorted(r.glob('field_scope_baseline_search_*'))[-1]
assert (t/'result.json').exists(),'Driver still active; do not open databases'
trial=json.loads((t/'result.json').read_text());s=Path(trial['session']);g=s/'goal_001'
assert (s/'session_result.json').exists() and (g/'result.json').exists(),'Recorders not closed'
def rows(path):return [json.loads(l) for l in path.read_text().splitlines() if l]
events=rows(g/'events.jsonl');native=rows(g/'native_samples.jsonl');positions=[x['position'] for x in native if isinstance(x.get('position'),list)]
statuses=[e for e in events if e.get('kind')=='plan_status'];following=[e for e in statuses if e.get('status')=='following']
stops=[];start=None;last=None
for x in native:
 if x.get('speed',math.inf)<.05:
  if start is None:start=x['wall_time']
  last=x['wall_time']
 else:
  if start is not None and last-start>=5.:stops.append(dict(start_wall=start,duration_sec=last-start))
  start=None
if start is not None and last-start>=5.:stops.append(dict(start_wall=start,duration_sec=last-start))
c=sqlite3.connect(next((g/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,typ) for i,n,typ in c.execute('select id,name,type from topics')}
counts={n:cnt for n,cnt in c.execute('select topics.name,count(messages.id) from topics left join messages on topics.id=messages.topic_id group by topics.id')}
def messages(topic):
 if topic not in topics:return []
 i,typ=topics[topic];kind=get_message(typ)
 return [(ts,deserialize_message(b,kind)) for ts,b in c.execute('select timestamp,data from messages where topic_id=? order by timestamp',(i,))]
decisions=[dict(received_ns=ts,**json.loads(msg.data)) for ts,msg in messages('/spot1/graphnav_planner/route_status')]
retired=[dict(received_ns=ts,**json.loads(msg.data)) for ts,msg in messages('/spot1/graphnav_planner/retired_exploration')]
acks=messages('/spot1/graphnav_planner/observed_route_endpoint')
odom_samples=[(ts/1e9,(m.pose.pose.position.x,m.pose.pose.position.y)) for ts,m in messages('/spot1/odom')]
ids=collections.defaultdict(list)
for d in decisions:ids[d.get('exploration_id')].append(d)
following_ids=collections.Counter(e.get('exploration_id') for e in following)
goal_drift=[]
for key,ds in ids.items():
 if key and len({tuple(x['goal']) for x in ds})>1:goal_drift.append(key)
vision=rows(s/'vision_participation.jsonl');final=next((z for z in reversed(vision) if z.get('kind')=='final'),None)
observations=[d for d in decisions if d.get('endpoint_observed')]
identity_progress={}
for key,ds in ids.items():
 if not key:continue
 first=ds[0]['received_ns']/1e9;finish=next((z['stamp_ns']/1e9 for z in retired if z.get('exploration_id')==key),float('inf'))
 samples=[x for x in native if first<=x['wall_time']<=finish and isinstance(x.get('position'),list)]
 destination=ds[0]['goal']
 identity_progress[key]=dict(goal=destination,first_received_wall=first,retired_wall=finish if math.isfinite(finish) else None,
  minimum_native_reference_distance_m=min((math.dist(x['position'][:2],destination) for x in samples),default=None),
  minimum_odom_reference_distance_m=min((math.dist(xy,destination) for ts,xy in odom_samples if first<=ts<=finish),default=None),
  body_distance_travelled_m=sum(math.dist(a['position'][:2],b['position'][:2]) for a,b in zip(samples,samples[1:])))
selection_audit=[]
for e in following:
 current=next((d for d in reversed(decisions) if d['received_ns']<=e['stamp_ns']),None)
 selection_audit.append(dict(event=e,latest_decision_id=current.get('exploration_id') if current else None))
fullpaths=[];raw=next((s/'sensor_interfaces').glob('*.db3'))
with sqlite3.connect(raw.as_uri()+'?mode=ro',uri=True) as sensor:
 pair=sensor.execute("select id,type from topics where name='/wildos/full_planned_path'").fetchone()
 if pair:
  kind=get_message(pair[1])
  for ts,b in sensor.execute('select timestamp,data from messages where topic_id=? order by timestamp',(pair[0],)):
   path=deserialize_message(b,kind);xy=[(z.pose.position.x,z.pose.position.y) for z in path.poses]
   if len(xy)<2:continue
   fullpaths.append(dict(received_ns=ts,path_length_m=sum(math.dist(a,b) for a,b in zip(xy,xy[1:])),displacement_m=math.dist(xy[0],xy[-1]),motion_sections=len(motion_sections(xy)),endpoint=xy[-1]))
report=dict(trial=trial,session=str(s),goal=json.loads((g/'result.json').read_text()),session_result=json.loads((s/'session_result.json').read_text()),
 status_counts=dict(collections.Counter(e.get('status') for e in statuses)),native_distance_m=sum(math.dist(a[:2],b[:2]) for a,b in zip(positions,positions[1:])),
 max_departure_m=max(math.dist(x[:2],positions[0][:2]) for x in positions) if positions else None,
 final_departure_m=math.dist(positions[0][:2],positions[-1][:2]) if positions else None,stationary_intervals_ge5s=stops,
 topic_counts=counts,route_decisions=len(decisions),exploration_ids={str(k):len(v) for k,v in ids.items()},following_ids=dict(following_ids),goal_drift_within_id=goal_drift,
 observed_route_ack_messages=len(acks),observed_decisions=len(observations),retirements=retired,retirement_reasons=dict(collections.Counter(x.get('reason') for x in retired)),identity_progress=identity_progress,
 vision_final=final,candidate_fullpaths=fullpaths,fullpath_scope='Candidate publications; not proof that all sections were executed',
 following_vs_latest_decision=selection_audit)
c.close();(p/'closed_review.json').write_text(json.dumps(report,indent=2))
print(json.dumps({k:v for k,v in report.items() if k not in ('following_vs_latest_decision','candidate_fullpaths','retirements')},indent=2))
