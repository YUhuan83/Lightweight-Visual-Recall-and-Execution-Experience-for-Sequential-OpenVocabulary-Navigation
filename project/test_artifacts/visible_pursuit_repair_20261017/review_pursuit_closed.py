"""Closed-only pursuit/geometry/action/source audit; never opens active bags."""
import collections,json,math,re,sqlite3,statistics
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

root=Path('/home/ly/visible_pursuit_repair_20261017')
trial=sorted(root.glob('field_yaw_left90_search_*'))[-1]
assert (trial/'result.json').is_file(),'Driver active'
driver=json.loads((trial/'result.json').read_text());s=Path(driver['session']);g=s/'goal_001'
assert (s/'session_result.json').is_file() and (g/'result.json').is_file(),'Recorder active'
goal=json.loads((g/'result.json').read_text())
def rows(path):
    with path.open() as f:
        for l in f:
            if l.strip():yield json.loads(l)
events=list(rows(g/'events.jsonl'));plans=[e for e in events if e.get('kind')=='plan_status']
query_wall=next(c['wall'] for c in driver['commands'] if c['text']=='green residential house')
native_samples=list(rows(g/'native_samples.jsonl'))
lo,hi=native_samples[0]['wall_time'],native_samples[-1]['wall_time']
issued={};native=[];control_plans=0;carrots=0
def key(d):return tuple(d[k] for k in ('sender_id','generation','sequence'))
for row in rows(s/'execution_observer/messages.jsonl'):
    topic=row['topic']
    if topic=='/wildos/native_control_command':
        frame=json.loads(row['message']['data']);issued[key(frame)]=frame
    elif topic=='/wildos/native_control_status':
        v=json.loads(row['message']['data'])
        if lo<=v['wall_time']<=hi:native.append(v)
    elif topic=='/received_global_plan':control_plans+=1
    elif topic=='/lookahead_point':carrots+=1
def stats(values):
    v=sorted(values)
    return dict(count=len(v),median=statistics.median(v),p95=v[int((len(v)-1)*.95)],max=v[-1]) if v else None
matched=0;bad=[];positive_unmatched=[]
for value in native:
    receipt=value.get('source_receipt')
    if receipt and receipt.get('applied') is True:
        frame=issued.get(key(receipt))
        if (frame is not None and all(receipt[k]==frame[k] for k in ('issued_wall_ns','expires_wall_ns','control'))
                and receipt['control']==value['sent'] and receipt['issued_wall_ns']<=receipt['applied_wall_ns']<receipt['expires_wall_ns']):matched+=1
        else:bad.append(value['wall_time'])
    elif value.get('control_acknowledged') is True and value['sent']['throttle']>0 and value['sent']['brake']<.9:
        positive_unmatched.append(value['wall_time'])

db=sqlite3.connect(next((g/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,t) for i,n,t in db.execute('select id,name,type from topics')}
def messages(topic):
    if topic not in topics:return []
    i,typ=topics[topic];kind=get_message(typ)
    return [(ts,deserialize_message(b,kind)) for ts,b in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(i,))]
masks=[dict(received_ns=t,stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,query=m.query)
       for t,m in messages('/spot1/object_mask')]
localizations=[dict(received_ns=t,data=json.loads(m.data)) for t,m in messages('/spot1/object_localization')]
confirmations=[dict(received_ns=t,data=json.loads(m.data)) for t,m in messages('/spot1/object_confirmation')]
hints=[dict(received_ns=t,data=json.loads(m.data)) for t,m in messages('/spot1/visible_target_approach')]
timing=[json.loads(m.data) for _,m in messages('/spot1/measurement_timing')]
sync=[v for v in timing if v.get('sensor')=='mask_lidar_sync']
db.close()
reobservations=[]
for i,e in enumerate(plans):
    if e['status']!='visible_target_reobserving':continue
    t=e['stamp_ns']/1e9
    next_state=next((x for x in plans[i+1:] if x['status'] in
        ('visible_target_approach','visible_target_reobservation_timeout','awaiting_graph_route',
         'target_observation_approach','object_found','planning_only')),None)
    recent=next((m for m in reversed(masks) if m['received_ns']<=e['stamp_ns']),None)
    reobservations.append(dict(start_query_relative_sec=t-query_wall,goal=e.get('goal'),
        mask_age_sec=(e['stamp_ns']-recent['stamp_ns'])/1e9 if recent else None,
        next_status=next_state['status'] if next_state else None,
        duration_sec=next_state['stamp_ns']/1e9-t if next_state else None))
controller_errors=[]
for line in (s/'stack.log').read_text().splitlines():
    mt=re.search(r'\[(\d+\.\d+)\]',line)
    if (mt and query_wall<=float(mt[1])<=hi and 'controller_server-' in line
            and any(w in line for w in ('[ERROR]','[WARN]','Aborting','collision','reverse a single'))):controller_errors.append(line)
report=dict(session=str(s),driver=str(trial),candidate=json.loads((root/'manifest.json').read_text())['sha256'],
    query_wall=query_wall,goal=goal,heading_check=driver.get('spawn_heading_check'),
    source_lifecycle_passed=driver.get('source_lifecycle',{}).get('passed'),
    status_counts=dict(collections.Counter(e['status'] for e in plans)),
    masks=len(masks),visible_hints=len(hints),localizations=localizations,confirmations=confirmations,
    sync_status_counts=dict(collections.Counter(v['status'] for v in sync)),
    sync_skew_ns=stats([abs(v['lidar_mask_skew_ns']) for v in sync]),
    reobservations=reobservations,controller_plan_messages=control_plans,lookahead_messages=carrots,
    controller_runtime_errors=controller_errors,
    source=dict(scope='All Native messages received by passive observer within goal sample interval; DDS loss possible; SDK ACK is not physical actuation',
        native_messages=len(native),issued_messages=len(issued),matched_receipts=matched,
        invalid_matching_receipts=bad,positive_control_without_valid_receipt=positive_unmatched,
        source_age_sec=stats([v['command_age'] for v in native]),
        callback_source_age_sec=stats([v['command_age']-v['command_receipt_age'] for v in native]),
        control_rpc_ms=stats([v['control_rpc_ms'] for v in native if v.get('control_rpc_ms') is not None]),
        watchdog_messages=sum(bool(v.get('watchdog_stop')) for v in native),
        actuator_failed_messages=sum(v.get('actuator_ok') is not True for v in native),
        telemetry_unhealthy_messages=sum(v.get('telemetry_ok') is not True for v in native)),
    visual_identity_review_pending=goal['state']=='automatic_arrival_pending_visual_review')
(root/'pursuit_closed_review.json').write_text(json.dumps(report,indent=2))
print(json.dumps({k:v for k,v in report.items() if k not in ('goal','localizations','confirmations','controller_runtime_errors','heading_check')},indent=2))
