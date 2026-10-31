"""Closed observer identity audit; no active bag reads and no physical-force claim."""
from pathlib import Path
import bisect,collections,hashlib,json,statistics
ROOT=Path('/home/ly/planning_phase_physical_20261018')
driver=sorted(ROOT.glob('field_yaw_left90_search_*'))[-1]
assert (driver/'result.json').is_file(),'Driver active'
trial=json.loads((driver/'result.json').read_text());session=Path(trial['session'])
assert (session/'session_result.json').is_file() and (session/'goal_001/result.json').is_file(),'Recorders active'
lo=next(x['wall'] for x in trial['commands'] if x['text']=='green residential house')
hi=lo+json.loads((session/'goal_001/result.json').read_text())['elapsed_s']
observer=session/'execution_observer/messages.jsonl'
sources={};statuses={};raw={};native=[];controller_ages=[]
def identity(frame):return tuple(frame.get(k) for k in ('sender_id','generation','sequence'))
def stats(values):
    values=sorted(values)
    return dict(count=len(values),min=values[0],median=statistics.median(values),
        p95=values[min(len(values)-1,int(len(values)*.95))],max=values[-1]) if values else None
for line in observer.open():
    row=json.loads(line);topic=row['topic'];wall=row['received_ros_ns']/1e9
    if topic=='/wildos/cmd_vel_stamped':
        msg=row['message'];stamp=msg['header']['stamp'];ns=stamp['sec']*10**9+stamp['nanosec']
        raw[ns]=msg
        if lo<=wall<=hi:controller_ages.append(wall-ns/1e9)
    elif topic in ('/wildos/native_control_command','/wildos/control_status','/wildos/native_control_status'):
        value=json.loads(row['message']['data'])
        if topic=='/wildos/native_control_command':sources[identity(value)]=value
        elif topic=='/wildos/control_status':statuses[identity(value.get('source_control') or {})]=value
        elif lo<=wall<=hi:native.append((wall,value))
violations=[];unmatched_positive=[];positive_source_ages=[];tracking=0;wrong_frames=[]
for key,status in statuses.items():
    if status['reason']!='tracking':continue
    tracking+=1;source=sources.get(key);stamp=status.get('controller_stamp_ros_ns',0)
    if not source:continue
    if not status.get('requires_stamped_command') or not stamp or source['expires_wall_ns']>stamp+500_000_000:
        violations.append(dict(key=key,status=status,source=source))
    if stamp in raw and raw[stamp]['header']['frame_id']!='spot1/nav_base_link':wrong_frames.append(stamp)
by_reason=collections.defaultdict(float);by_sent=collections.defaultdict(float)
stopped_samples=0;exact=0;covered=0.;max_applied_age=0.
native.sort(key=lambda x:x[0])
for (wall,value),(nextwall,_) in zip(native,native[1:]):
    dt=min(.2,max(0.,nextwall-wall));covered+=dt
    receipt=value.get('source_receipt') or {};key=identity(receipt)
    source=sources.get(key);status=statuses.get(key)
    matched=bool(source and status and receipt.get('applied') is True
        and all(receipt.get(k)==source.get(k) for k in ('issued_wall_ns','expires_wall_ns','control'))
        and receipt['control']==value['sent'])
    positive=value['sent']['throttle']>0. and value['sent']['brake']<.9
    if positive:
        if not matched:unmatched_positive.append(dict(wall=wall,key=key))
        else:
            stamp=status.get('controller_stamp_ros_ns',0)
            age=(receipt['applied_wall_ns']-stamp)/1e9
            positive_source_ages.append(age);max_applied_age=max(max_applied_age,age)
            if not stamp or receipt['applied_wall_ns']>=stamp+500_000_000:
                violations.append(dict(key=key,applied=receipt,status=status))
    if value['speed']<.05:
        stopped_samples+=1;exact+=matched
        by_reason[status['reason'] if matched else 'unmatched_or_not_applied']+=dt
        sent=value['sent'];category=('braking' if sent['brake']>.1 else
            'positive_throttle' if sent['throttle']>.1 and sent['brake']<=.01 else 'coasting_or_low_torque')
        by_sent[category]+=dt
out=dict(candidate=json.loads((ROOT/'manifest.json').read_text())['sha256'],
    scope=__doc__,observer_sha256=hashlib.sha256(observer.read_bytes()).hexdigest(),
    native_samples=len(native),covered_seconds=covered,controller_stamped_samples=len(raw),
    controller_dds_source_age_sec=stats(controller_ages),tracking_frames=tracking,
    positive_source_to_applied_age_sec=stats(positive_source_ages),
    source_deadline_violations=violations,wrong_controller_frames=wrong_frames,
    unmatched_positive=unmatched_positive,stopped_samples=stopped_samples,
    stopped_exact_identity_matches=exact,stopped_seconds_by_source_reason=dict(by_reason),
    stopped_seconds_by_sent_control=dict(by_sent))
(ROOT/'controller_chain_review.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(out,indent=2))
