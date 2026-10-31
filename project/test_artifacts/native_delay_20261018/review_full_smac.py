"""Closed full-query review in the observer/goal shared Python monotonic domain."""
import argparse,bisect,collections,json,math,re,sqlite3,statistics
from pathlib import Path

def stats(a):
    a=sorted(a)
    return dict(n=len(a),min=a[0],median=statistics.median(a),p95=a[int(.95*len(a))],max=a[-1]) if a else None

def review(session):
    assert (session/'session_result.json').is_file(),'Never read active session DB'
    goal=json.loads((session/'goal_001/result.json').read_text())
    lo=goal['started'];hi=lo+goal['elapsed_s']
    rows=[json.loads(l) for l in (session/'execution_observer/messages.jsonl').open()]
    offset=statistics.median((r['received_ros_ns']-r['received_monotonic_ns'])/1e9 for r in rows)
    def key(v):return tuple(v.get(k) for k in ('sender_id','generation','sequence'))
    sources={};controls={};native=[];plans=[];odom=[];controller_paths=0
    for r in rows:
        t=r['received_monotonic_ns']/1e9;topic=r['topic'];msg=r['message']
        if topic=='/spot1/odom' and lo<=t<=hi:odom.append(msg)
        if topic=='/received_global_plan' and lo<=t<=hi:controller_paths+=1
        if topic not in ('/wildos/native_control_command','/wildos/control_status','/wildos/native_control_status','/wildos/plan_status'):continue
        v=json.loads(msg['data'])
        if topic=='/wildos/native_control_command':sources[key(v)]=v
        elif topic=='/wildos/control_status':controls[key(v.get('source_control') or {})]=v
        elif topic=='/wildos/plan_status':plans.append((t,v))
        elif lo<=t<=hi:native.append((t,v))
    assert len(native)>2
    pt=[t for t,_ in plans];covered=0.;stopped=0.;matches=0
    reasons=collections.defaultdict(float);sent_reasons=collections.defaultdict(float);states=collections.defaultdict(float)
    positive=[];bad=[];wrong=[];stops=[];stop_at=None
    for (t,v),(next_t,_) in zip(native,native[1:]):
        dt=min(.2,max(0.,next_t-t));covered+=dt
        receipt=v.get('source_receipt') or {};frame=sources.get(key(receipt));c=controls.get(key(receipt))
        matched=bool(frame and c and receipt.get('applied') is True and all(receipt.get(k)==frame.get(k) for k in ('issued_wall_ns','expires_wall_ns','control')) and receipt['control']==v['sent'])
        if v['sent']['throttle']>0 and v['sent']['brake']<.9:
            if not matched:bad.append(t)
            else:
                stamp=c.get('controller_stamp_ros_ns',0);positive.append((receipt['applied_wall_ns']-stamp)/1e9)
                if not stamp or receipt['applied_wall_ns']>=stamp+500_000_000:wrong.append(t)
        if v['speed']<.05:
            if stop_at is None:stop_at=t
            stopped+=dt;matches+=matched
            reasons[c['reason'] if matched else 'unmatched_or_not_applied']+=dt
            sent=v['sent'];sent_reasons['braking' if sent['brake']>.1 else 'positive_throttle' if sent['throttle']>.1 and sent['brake']<=.01 else 'coasting_or_low_torque']+=dt
            i=bisect.bisect_right(pt,t)-1;states[plans[i][1]['status'] if i>=0 else 'before_first_status']+=dt
        elif stop_at is not None:
            stops.append(dict(start=stop_at-lo,duration=t-stop_at));stop_at=None
    if stop_at is not None:stops.append(dict(start=stop_at-lo,duration=native[-1][0]-stop_at))
    positions=[v['position'][:2] for _,v in native]
    requests=0;results=collections.Counter();late=0;planner_errors=collections.Counter();raw_fallback=0
    for line in (session/'stack.log').read_text(errors='replace').splitlines():
        m=re.search(r'\[(\d+\.\d+)\]',line)
        if not m or not lo+offset<=float(m[1])<=hi+offset:continue
        requests+='ComputePath request sent' in line;late+='Late ComputePath result ignored' in line
        raw_fallback+='smoothed path rejected; validating same-search raw path' in line
        if (m:=re.search(r'ComputePath result: epoch=(\d+) candidate=(\d+) status=(\d+) poses=(\d+)',line)):results[int(m[3])]+=1
        for marker in ('No safe bounded continuous-start plan','Raw path violates','no valid path','maximum iterations','bounded command arc collision','source_lifetime'):
            if marker in line:planner_errors[marker]+=1
    # Recorded counters in the identical canonical wall interval. Decode only
    # target observations and public route status, after recorder/session close.
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    counts=collections.Counter();target_counts=collections.Counter();route_rows=[]
    db=next((session/'goal_001/interfaces').glob('*.db3'))
    con=sqlite3.connect('file:'+str(db)+'?mode=ro',uri=True)
    topics={i:(name,typ) for i,name,typ in con.execute('SELECT id,name,type FROM topics')}
    selected={'/spot1/object_mask','/spot1/object_localization','/spot1/object_confirmation','/spot1/visible_target_approach','/spot1/graphnav_planner/route_status'}
    try:
        for topic_id,timestamp,data in con.execute('SELECT topic_id,timestamp,data FROM messages WHERE timestamp BETWEEN ? AND ? ORDER BY timestamp',(round((lo+offset)*1e9),round((hi+offset)*1e9))):
            name,typ=topics[topic_id];counts[name]+=1
            if name not in selected:continue
            msg=deserialize_message(data,get_message(typ))
            if name=='/spot1/graphnav_planner/route_status':
                value=json.loads(msg.data);route_rows.append((timestamp/1e9,value));continue
            query=getattr(msg,'query',None)
            if hasattr(msg,'data'):
                try:query=json.loads(msg.data).get('query',query)
                except (ValueError,AttributeError):pass
            if query==goal['query']:target_counts[name]+=1
    finally:con.close()
    report=dict(scope=__doc__,goal=goal,goal_started_wall=lo+offset,native_samples=len(native),covered_sec=covered,
        distance_m=sum(math.dist(a,b) for a,b in zip(positions,positions[1:])),max_departure_m=max(math.dist(p,positions[0]) for p in positions),
        stopped_sec=stopped,stopped_by_source_reason=dict(reasons),stopped_by_sent_control=dict(sent_reasons),
        stopped_by_latest_observed_status=dict(states),matched_stopped_samples=matches,long_stops=[s for s in stops if s['duration']>=5],
        positive_source_to_applied_age=stats(positive),unmatched_positive=bad,source_deadline_violations=wrong,
        native_callback_source_age=stats([v['command_age']-v['command_receipt_age'] for _,v in native if v['command_receipt_age'] is not None]),
        watchdog_samples=sum(v['watchdog_stop'] for _,v in native),missing_receipts=sum(v.get('source_receipt') is None for _,v in native),
        planner_requests=requests,current_planner_result_status_counts=dict(results),ignored_late_results=late,
        smac_raw_fallback_logs=raw_fallback,planner_error_log_counts=dict(planner_errors),controller_transformed_paths=controller_paths,
        plan_status_counts=dict(collections.Counter(v['status'] for t,v in plans if lo<=t<=hi)),recorded_topic_counts=dict(counts),target_observation_counts=dict(target_counts),
        graph_routes=dict(count=len(route_rows),complete=sum(bool(v.get('complete')) for _,v in route_rows),max_publication_gap_sec=max([b[0]-a[0] for a,b in zip(route_rows,route_rows[1:])],default=None)),
        limits='Public status association is approximate, not planning CPU time. Request/result serials are not forced matched. Raw fallback log includes all smoothing rejection reasons, not only boundary yaw. One run is not stability or memory-increment acceptance.')
    assert abs(sum(reasons.values())-stopped)<1e-6 and abs(sum(sent_reasons.values())-stopped)<1e-6
    (session/'full_query_review.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('recorded_topic_counts','long_stops','goal')},indent=2))
    return report
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('session',type=Path);args=p.parse_args();review(args.session)
