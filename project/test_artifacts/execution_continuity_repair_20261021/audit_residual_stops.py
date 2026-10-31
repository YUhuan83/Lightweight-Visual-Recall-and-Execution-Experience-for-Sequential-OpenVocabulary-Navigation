"""Read closed bags only; align source stamps and never infer CPU time from status."""
from pathlib import Path
import bisect
from collections import Counter
import hashlib
import json
import math
import sqlite3
import sys
from rclpy.serialization import deserialize_message
from std_msgs.msg import String

trial=Path(sys.argv[1]); out=Path(sys.argv[2]);out.mkdir(parents=True,exist_ok=True)
session=next(trial.glob('sessions/*'));bag=next(session.glob('goal_001/interfaces/*.db3'))
db=sqlite3.connect('file:'+str(bag)+'?mode=ro',uri=True)
def read(topic):
    return [(t/1e9,deserialize_message(b,String).data) for t,b in db.execute(
        'select m.timestamp,m.data from messages m join topics t on m.topic_id=t.id where t.name=? order by m.timestamp',(topic,))]
q0=next(t for t,v in read('/spot1/object_search/query') if v=='green residential house');q1=q0+120
native=[(json.loads(v)['wall_time'],dict(json.loads(v),bag_time=t)) for t,v in read('/wildos/native_control_status')]
control=[(json.loads(v)['stamp_ns']/1e9,json.loads(v)) for t,v in read('/wildos/control_status')]
plan=[(json.loads(v)['stamp_ns']/1e9,json.loads(v)) for t,v in read('/wildos/plan_status')]
native.sort();control.sort();plan.sort()
def latest(rows,t):
    i=bisect.bisect_right([v[0] for v in rows],t)-1
    return rows[i] if i>=0 else (None,{})
cuts=sorted(set([q0,q1]+[t for rows in [native,control,plan] for t,_ in rows if q0<t<q1]))
intervals=[];by_control=Counter();by_plan=Counter();cross=Counter()
for a,b in zip(cuts,cuts[1:]):
    nt,n=latest(native,a);ct,c=latest(control,a);pt,p=latest(plan,a)
    if nt is None or a-nt>.5:continue
    if n['speed']<.1:
        dt=b-a;reason=c.get('reason','unknown');state=p.get('status','unknown')
        by_control[reason]+=dt;by_plan[state]+=dt;cross[(reason,state)]+=dt
        intervals.append(dict(start=a,end=b,native=n,control=c,plan=p))
runs=[]
for item in intervals:
    if not runs or abs(runs[-1]['end']-item['start'])>1e-6:
        runs.append(dict(start=item['start'],end=item['end'],items=[item]))
    else:runs[-1]['end']=item['end'];runs[-1]['items'].append(item)
details=[]
for run in sorted(runs,key=lambda v:v['end']-v['start'],reverse=True):
    dur=run['end']-run['start']
    if dur<3:continue
    c=Counter();p=Counter()
    for row in run['items']:
        dt=row['end']-row['start'];c[row['control'].get('reason','unknown')]+=dt;p[row['plan'].get('status','unknown')]+=dt
    _,last_native=latest(native,run['end'])
    timeline=[dict(relative_s=t-q0,**{k:v.get(k) for k in ['status','reason','detail','request_serial','candidate_index','candidate_uuid','goal','graph_goal','exploration_goal','retirement_reason','background'] if k in v})
              for t,v in plan if run['start']-2<=t<=run['end'] and v['status']!='following']
    details.append(dict(start_s=run['start']-q0,end_s=run['end']-q0,duration_s=dur,
        net_m=math.dist(run['items'][0]['native']['position'][:2],last_native['position'][:2]),
        control_s=dict(c),plan_s=dict(p),timeline=timeline))
health=[]
for t,v in plan:
    if not q0<=t<=q1 or v['status']!='execution_unavailable':continue
    nt,n=latest(native,t);ct,c=latest(control,t)
    recorded=[(tt,row) for tt,row in native if tt<=t and row['bag_time']<=t]
    rt,r=recorded[-1] if recorded else (None,{})
    health.append(dict(relative_s=t-q0,reason=v.get('reason'),
        latest_native_source_age_s=t-nt,latest_telemetry_total_age_s=t-nt+n.get('telemetry_age_sec',0),
        native_telemetry_ok=n.get('telemetry_ok'),native_poll_ms=n.get('poll_roundtrip_ms'),
        already_recorded_source_age_s=t-rt if rt is not None else None,
        speed_node_status_age_s=t-ct,speed_node_native_source_age_s=t-c.get('native_source_wall_time',t),
        speed_node_native_health=c.get('native_health_ok'),speed_node_reason=c.get('reason')))
gear=[];current=None
for i,(t,c) in enumerate(control):
    end=control[i+1][0] if i+1<len(control) else q1;a=max(q0,t);b=min(q1,end)
    if b<=a:continue
    if c.get('reason')=='gear_change':
        if current is None:current=dict(start_s=a-q0,end_s=b-q0)
        else:current['end_s']=b-q0
    elif current is not None:gear.append(current);current=None
if current:gear.append(current)
native_times=[t for t,_ in native if q0<=t<=q1]
source_bad=Counter(v.get('source_rejection') for t,v in native if q0<=t<=q1)
report=dict(scope=__doc__,query_start_wall=q0,window_s=120,stationary_s=sum(by_control.values()),
    stationary_by_control=dict(by_control),stationary_by_plan=dict(by_plan),
    stationary_cross=[dict(control=k[0],plan=k[1],seconds=v) for k,v in cross.most_common()],
    stops_over_3s=details,gear_episodes=gear,health_transitions=health,
    native_sample_gap_max_s=max(b-a for a,b in zip(native_times,native_times[1:])),
    source_rejection_records=dict(source_bad),bag=dict(path=str(bag),sha256=hashlib.sha256(bag.read_bytes()).hexdigest()),
    complete_navigation_passed=False)
assert abs(sum(by_control.values())-sum(by_plan.values()))<1e-6
assert abs(sum(by_control.values())-sum(cross.values()))<1e-6
(out/'residual_stops.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:report[k] for k in ['stationary_s','stationary_by_control','stationary_by_plan','health_transitions']},indent=2))
