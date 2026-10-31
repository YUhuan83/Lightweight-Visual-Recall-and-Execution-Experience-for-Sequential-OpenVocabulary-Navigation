"""Separate pre-shift software dwell from actual gearbox response in closed trials."""
import bisect,hashlib,json,sqlite3
from pathlib import Path
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
root=Path('/home/ly/execution_continuity_review_20261022/v25_fast_shift')
output=[]
for name in sorted(p.name for p in (root/'full').glob('trial_*')):
    trial=root/'full'/name;assert json.loads((trial/'result.json').read_text())['closed']
    goal=next(trial.glob('sessions/*/goal_001'))
    db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
    tid=db.execute('select id from topics where name=?',('/wildos/control_status',)).fetchone()[0]
    data=sorted({d['stamp_ns']:d for blob, in db.execute('select data from messages where topic_id=?',(tid,)) for d in [json.loads(deserialize_message(blob,String).data)]}.values(),key=lambda d:d['stamp_ns']);db.close()
    native=[json.loads(line) for line in (goal/'native_samples.jsonl').read_text().splitlines()]
    groups=[];current=[]
    for d in data:
        if d['reason']=='gear_change':
            if current and (d['stamp_ns']-current[-1]['stamp_ns'])/1e9>.3:groups.append(current);current=[]
            current.append(d)
        elif current:groups.append(current);current=[]
    if current:groups.append(current)
    rows=[]
    for group in groups:
        a,b=group[0]['stamp_ns']/1e9,group[-1]['stamp_ns']/1e9
        target=group[0]['pending_gear']
        sent=next((n for n in native if a<=n['wall_time']<=b+1 and n.get('sent',{}).get('gear')==target),None)
        actual=next((n for n in native if sent and sent['wall_time']<=n['wall_time']<=b+1 and isinstance(n.get('actual',{}).get('gear_index'),(int,float)) and n['actual']['gear_index']*(-1 if target<0 else 1)>0),None)
        rows.append(dict(start_ns=group[0]['stamp_ns'],duration_s=b-a,first_speed=group[0]['measured_speed'],
            low_speed_wait_s=sum((y['stamp_ns']-x['stamp_ns'])/1e9 for x,y in zip(group,group[1:]) if x['measured_speed']<=.15),
            request_signs=sorted({1 if x['target_speed']>0 else -1 for x in group}),
            min_command_lifetime_s=min((x['controller_deadline_remaining_sec'] for x in group if x.get('controller_deadline_remaining_sec') is not None),default=None),
            target_selector=target,first_target_sent_after_start_s=sent['wall_time']-a if sent else None,
            first_matching_actual_after_sent_s=actual['wall_time']-sent['wall_time'] if actual else None,
            actual_gear=actual.get('actual',{}).get('gear') if actual else None))
    output.append(dict(trial=str(trial),segments=rows,total_low_speed_wait_s=sum(r['low_speed_wait_s'] for r in rows)))
encoded=json.dumps(output,sort_keys=True).encode()
(root/'shift_wait_audit.json').write_bytes(encoded)
print(json.dumps(dict(sha256=hashlib.sha256(encoded).hexdigest(),rounds=[dict(trial=r['trial'],shifts=len(r['segments']),low_speed_wait_s=r['total_low_speed_wait_s'],first_three=r['segments'][:3]) for r in output])))

