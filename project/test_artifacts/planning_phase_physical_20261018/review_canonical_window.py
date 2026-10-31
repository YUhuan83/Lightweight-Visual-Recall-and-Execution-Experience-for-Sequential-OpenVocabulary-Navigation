"""Use goal.started and observer Python monotonic, not console input wall time."""
import bisect,collections,hashlib,json,math,re,statistics
from pathlib import Path
root=Path(__file__).resolve().parent
closed=root/'closed_evidence';session=closed/'field_results/session_20261018_151920'
goal=json.loads((session/'goal_001/result.json').read_text())
assert (session/'session_result.json').is_file()
lo=goal['started'];hi=lo+goal['elapsed_s']
rows=[json.loads(l) for l in (session/'execution_observer/messages.jsonl').open()]
offset=statistics.median((r['received_ros_ns']-r['received_monotonic_ns'])/1e9 for r in rows)
sources={};controls={};native=[];plans=[]
def key(v):return tuple(v.get(k) for k in ('sender_id','generation','sequence'))
for r in rows:
    t=r['received_monotonic_ns']/1e9;topic=r['topic']
    if topic in ('/wildos/native_control_command','/wildos/control_status','/wildos/native_control_status','/wildos/plan_status'):
        v=json.loads(r['message']['data'])
        if topic=='/wildos/native_control_command':sources[key(v)]=v
        elif topic=='/wildos/control_status':controls[key(v.get('source_control') or {})]=v
        elif topic=='/wildos/plan_status':plans.append((t,v))
        elif lo<=t<=hi:native.append((t,v))
def stats(a):
    a=sorted(a)
    return dict(n=len(a),min=a[0],median=statistics.median(a),p95=a[int(.95*len(a))],max=a[-1]) if a else None
pt=[t for t,_ in plans];seconds=collections.defaultdict(float);reasons=collections.defaultdict(float);states=collections.defaultdict(float)
covered=0.;bad=[];wrong=[];positive=[];stopped=0.;matches=0
for (t,v),(next_t,_) in zip(native,native[1:]):
    dt=min(.2,max(0,next_t-t));covered+=dt
    receipt=v.get('source_receipt') or {};frame=sources.get(key(receipt));c=controls.get(key(receipt))
    matched=bool(frame and c and receipt.get('applied') is True and all(receipt.get(k)==frame.get(k) for k in ('issued_wall_ns','expires_wall_ns','control')) and receipt['control']==v['sent'])
    if v['sent']['throttle']>0 and v['sent']['brake']<.9:
        if not matched:bad.append(t)
        else:
            stamp=c.get('controller_stamp_ros_ns',0)
            positive.append((receipt['applied_wall_ns']-stamp)/1e9)
            if not stamp or receipt['applied_wall_ns']>=stamp+500_000_000:wrong.append(t)
    if v['speed']<.05:
        stopped+=dt;matches+=matched
        reasons[c['reason'] if matched else 'unmatched_or_not_applied']+=dt
        control=v['sent'];seconds['braking' if control['brake']>.1 else 'positive_throttle' if control['throttle']>.1 and control['brake']<=.01 else 'coasting_or_low_torque']+=dt
        i=bisect.bisect_right(pt,t)-1;states[plans[i][1]['status'] if i>=0 else 'before_first_status']+=dt
positions=[v['position'][:2] for _,v in native]
shadow_root=root.parent/'execution_wait_audit_20261018/startup_receipt/search_closed_evidence/diagnostics'
shadow=[json.loads(l) for l in next(shadow_root.glob('search_shadow_*.jsonl')).open()]
shadow_age={kind:stats([(r['wall_ns']-r['data']['issued_wall_ns'])/1e9 for r in shadow if r['kind']==kind and lo<=r['mono_ns']/1e9<=hi]) for kind in ('reliable1','reliable10','best1')}
requests=0;results=collections.Counter();late=0
for line in (session/'stack.log').read_text().splitlines():
    m=re.search(r'\[(\d+\.\d+)\]',line)
    if not m or not lo+offset<=float(m[1])<=hi+offset:continue
    requests+='ComputePath request sent' in line;late+='Late ComputePath result ignored' in line
    if (m:=re.search(r'ComputePath result: epoch=(\d+) candidate=(\d+) status=(\d+) poses=(\d+)',line)):results[int(m[3])]+=1
driver=json.loads((closed/'field_yaw_left90_search_20261018_151916/result.json').read_text())
input_wall=next(c['wall'] for c in driver['commands'] if c['text']=='green residential house')
report=dict(scope=__doc__,goal_started_monotonic=lo,goal_ended_monotonic=hi,goal_started_wall=lo+offset,
    console_input_to_goal_start_sec=lo+offset-input_wall,native_samples=len(native),covered_sec=covered,
    distance_m=sum(math.dist(a,b) for a,b in zip(positions,positions[1:])),max_departure_m=max(math.dist(p,positions[0]) for p in positions),
    stopped_sec=stopped,stopped_by_source_reason=dict(reasons),stopped_by_sent_control=dict(seconds),
    stopped_by_latest_observed_status=dict(states),matched_stopped_samples=matches,
    positive_source_to_applied_age=stats(positive),unmatched_positive=bad,source_deadline_violations=wrong,
    native_callback_source_age=stats([v['command_age']-v['command_receipt_age'] for _,v in native if v['command_receipt_age'] is not None]),
    watchdog_samples=sum(v['watchdog_stop'] for _,v in native),missing_receipts=sum(v.get('source_receipt') is None for _,v in native),
    shadow_source_ages=shadow_age,planner_requests=requests,current_planner_result_status_counts=dict(results),ignored_late_results=late,
    limits='Latest public status is approximate attribution, not private planning CPU/lock time. Native age weighted by status samples, not unique callbacks. Request/result logs lack common serial; no forced matching.')
assert abs(sum(reasons.values())-stopped)<1e-6 and abs(sum(seconds.values())-stopped)<1e-6
assert not bad and not wrong
(root/'canonical_window_review.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
