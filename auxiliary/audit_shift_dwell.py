"""Read closed bags to distinguish configured direction dwell from fresh-command loss."""
import hashlib,json,math,sqlite3
from pathlib import Path
from std_msgs.msg import String
from rclpy.serialization import deserialize_message
base=Path('/home/ly/execution_continuity_review_20261022/v22_odometry_ingress/full')
output=[]
for trial in [base/'trial_20261024_165941',base/'trial_20261024_170417']:
    assert json.loads((trial/'result.json').read_text())['closed']
    goal=next(trial.glob('sessions/*/goal_001'))
    db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
    tid=db.execute('select id from topics where name=?',('/wildos/control_status',)).fetchone()[0]
    data=sorted({d['stamp_ns']:d for b, in db.execute('select data from messages where topic_id=?',(tid,)) for d in [json.loads(deserialize_message(b,String).data)]}.values(),key=lambda d:d['stamp_ns']);db.close()
    segments=[];current=[]
    for d in data:
        if d['reason']=='gear_change':
            if current and (d['stamp_ns']-current[-1]['stamp_ns'])/1e9>.3:segments.append(current);current=[]
            current.append(d)
        elif current:segments.append(current);current=[]
    if current:segments.append(current)
    rows=[]
    for group in segments:
        start,end=group[0]['stamp_ns'],group[-1]['stamp_ns']
        next_status=next((d for d in data if d['stamp_ns']>end),None)
        low=next((d for d in group if d['measured_speed']<=.15),None)
        rows.append(dict(start_ns=start,duration_s=(end-start)/1e9,first_speed=group[0]['measured_speed'],
            first_low_speed_after_s=(low['stamp_ns']-start)/1e9 if low else None,
            low_speed_wait_s=sum((b['stamp_ns']-a['stamp_ns'])/1e9 for a,b in zip(group,group[1:]) if a['measured_speed']<=.15),
            requested_signs=sorted({1 if d['target_speed']>0 else -1 for d in group}),
            pending_gears=sorted({d['pending_gear'] for d in group if d.get('pending_gear') is not None}),
            minimum_command_lifetime_s=min((d['controller_deadline_remaining_sec'] for d in group if d.get('controller_deadline_remaining_sec') is not None),default=None),
            next_reason=next_status['reason'] if next_status else None,
            next_gear=next_status.get('current_gear') if next_status else None))
    output.append(dict(trial=str(trial),segments=rows,total_low_speed_gear_wait_s=sum(r['low_speed_wait_s'] for r in rows)))
encoded=json.dumps(output,sort_keys=True).encode();target=base.parent/'shift_dwell_audit.json';target.write_bytes(encoded)
print(json.dumps(dict(sha256=hashlib.sha256(encoded).hexdigest(),summary=[dict(trial=d['trial'],segments=len(d['segments']),low_speed_wait_s=d['total_low_speed_gear_wait_s']) for d in output])))
