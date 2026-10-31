"""Read-only physical telemetry versus video and control state in a closed trial."""
import sqlite3,json,math,collections,bisect,pathlib,hashlib
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
root=pathlib.Path(__file__).resolve().parents[1]
trial=root/'full/trial_20261020_225946'
result=json.loads((trial/'result.json').read_text()); assert result['closed']
start=result['video_capture']['started_wall_ns']/1e9
bag=pathlib.Path(result['session'])/'goal_001/interfaces/interfaces_0.db3'
assert (bag.parent/'metadata.yaml').exists()
c=sqlite3.connect('file:'+str(bag)+'?mode=ro',uri=True)
topics={n:i for i,n in c.execute('select id,name from topics')}
keys=['native_control_status','control_status','plan_status','execution_feedback']
rows={}
for key in keys:
    out=[]
    for t,b in c.execute('select timestamp,data from messages where topic_id=? order by timestamp',(topics['/wildos/'+key],)):
        v=json.loads(deserialize_message(b,String).data)
        wall=v.get('wall_time',t/1e9) if key=='native_control_status' else t/1e9
        out.append((wall-start,v))
    rows[key]=out
native=rows['native_control_status']; control=rows['control_status']; plans=rows['plan_status']
ctl_times=[t for t,v in control]; plan_times=[t for t,v in plans]
def at(records,times,t):
    i=bisect.bisect_right(times,t)-1
    return records[i][1] if i>=0 else {}
def xy(v): return v['position'][:2]
def d(a,b): return math.dist(a[:2],b[:2])
def summary(lo,hi):
    selected=[(t,v) for t,v in native if lo<=t<hi and isinstance(v.get('position'),list)]
    if not selected:return {}
    weights=collections.Counter(); reasons=collections.Counter(); planning=collections.Counter(); maxgap=0
    longest=[]; run=None
    for (t,v),(tn,vn) in zip(native,native[1:]):
        dt=max(0,min(tn,hi)-max(t,lo))
        if dt<=0:continue
        maxgap=max(maxgap,tn-t)
        valid=v.get('telemetry_ok') is True and 0<=v.get('telemetry_age_sec',999)<=.5 and tn-t<=.5
        weights['observed_sec' if valid else 'unobserved_sec']+=dt
        if not valid:continue
        stopped=v.get('speed',999)<.1
        weights['speed_below_0.1_sec' if stopped else 'moving_sec']+=dt
        actual=v.get('actual') or {}; sent=v.get('sent') or {}
        if stopped:
            if actual.get('brake',0)>=.5:weights['stopped_actual_brake_sec']+=dt
            if actual.get('throttle',0)>.05:weights['stopped_actual_throttle_sec']+=dt
            if sent.get('brake',0)>=.5:weights['stopped_sent_brake_sec']+=dt
            cv=at(control,ctl_times,t)
            reasons[cv.get('reason','missing')]+=dt
            pv=at(plans,plan_times,t)
            planning[str(pv.get('status'))+':'+str(pv.get('reason'))]+=dt
        if v.get('source_receipt') is None:weights['source_receipt_missing_sec']+=dt
        if v.get('watchdog_stop'):weights['native_watchdog_sec']+=dt
    first=xy(selected[0][1]); last=xy(selected[-1][1]); origin=xy(native[0][1])
    sampled=[]
    for t,v in selected:
        if not sampled or t-sampled[-1][0]>=.5:sampled.append((t,v))
    return dict(range_sec=[lo,hi],first_xy=first,last_xy=last,net_m=d(first,last),distance_from_trial_start_m=d(origin,last),
        max_distance_from_trial_start_m=max(d(origin,xy(v)) for t,v in selected),
        sampled_path_length_m=sum(d(xy(a[1]),xy(b[1])) for a,b in zip(sampled,sampled[1:])),
        x_range=[min(v['position'][0] for t,v in selected),max(v['position'][0] for t,v in selected)],
        y_range=[min(v['position'][1] for t,v in selected),max(v['position'][1] for t,v in selected)],
        duration_weights=dict(weights),stationary_control_reasons_sec=dict(reasons),stationary_plan_states_sec=dict(planning),
        max_native_wall_gap_sec=maxgap,samples=len(selected),unique_sim_time=len(set(v.get('sim_time') for t,v in selected)))
intervals=[]; run=None
for (t,v),(tn,vn) in zip(native,native[1:]):
    good=v.get('telemetry_ok') is True and 0<=v.get('telemetry_age_sec',999)<=.5 and tn-t<=.5 and v.get('speed',999)<.1
    if good and run is None:run=t
    if not good and run is not None:
        if t-run>=5:intervals.append(summary(run,t))
        run=None
if run is not None and native[-1][0]-run>=5:intervals.append(summary(run,native[-1][0]))
events=[]; last=None
for t,v in plans:
    state=(v.get('status'),v.get('reason'))
    if state==last:continue
    last=state
    if state[0] in ('execution_unavailable','execution_recovered','replanning_after_maneuver','maneuver_done','route_recovery_exhausted'):
        i=bisect.bisect_right([a for a,b in native],t)-1
        nv=native[max(0,i)][1]
        cv=at(control,ctl_times,t)
        events.append(dict(video_sec=t,plan=v,native={k:nv.get(k) for k in ('wall_time','sim_time','position','speed','telemetry_age_sec','control_rpc_ms','poll_roundtrip_ms','source_receipt','source_rejection','watchdog_stop','sent','actual')},control=cv))
report=dict(scope=__doc__,trial=str(trial),video_wall_start=start,closed=True,native_samples=len(native),
    video_duration=result['video_capture']['duration_s'],overall=summary(0,426.1),minutes=[summary(t,min(t+60,426.1)) for t in range(0,426,60)],
    stationary_intervals=intervals,events=events,complete_navigation_passed=False,
    caveat='Duration weighting uses valid SDK telemetry and gaps <=0.5s; observed speed<0.1m/s is near-stationary, not mathematically zero. Native world position, not odometry integration. Status gaps are not counted as zero speed.')
out=root/'motion_audit/physical_motion_review.json';out.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k not in ('events','stationary_intervals')},indent=2))
print('STOP_INTERVALS',[(round(x['range_sec'][0],2),round(x['range_sec'][1],2),x['stationary_control_reasons_sec']) for x in intervals])
print('FIRST_EVENTS',json.dumps(events[:5],indent=2))
