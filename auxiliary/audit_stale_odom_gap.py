"""Compare bridge stale-odom reports with source and bag receipt timestamps."""
import json
import sqlite3
import sys
from pathlib import Path
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
sys.path.insert(0,'/home/ly/execution_continuity_review_20261022/v18_comparison_bound/candidate/nav2_adapter')
from wildos_nav2_adapter.input_quality import valid_pose, measurement_age

trial=Path(sys.argv[1])
assert json.loads((trial/'result.json').read_text())['closed']
goal=next(trial.glob('sessions/*/goal_001'))
assert (goal/'stop.json').exists()
events=[json.loads(line) for line in (goal/'events.jsonl').read_text().splitlines()]
plans=[d for d in events if d.get('kind')=='plan_status']
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
tid=db.execute('select id from topics where name=?',('/spot1/odom',)).fetchone()[0]
result=[]
for event in plans:
    if event.get('status')!='stale_odom':
        continue
    t=event['stamp_ns']
    samples=[]
    for receipt,blob in db.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp',(tid,t-2_000_000_000,t+2_000_000_000)):
        msg=deserialize_message(blob,Odometry)
        source=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        samples.append(dict(receipt_ns=receipt,source_ns=source,receipt_minus_source_s=(receipt-source)/1e9,
            frame=msg.header.frame_id,valid_pose=valid_pose(msg.pose.pose),
            admitted_at_event=(msg.header.frame_id=='spot1/odom' and valid_pose(msg.pose.pose)
                               and measurement_age(msg,t,1.) is not None)))
    previous=[s for s in samples if s['receipt_ns']<=t]
    preceding=[d for d in plans if t-5_000_000_000<=d['stamp_ns']<t]
    recent_planned=[d for d in preceding if d['status']=='planned']
    last=previous[-1] if previous else None
    result.append(dict(event=event,previous_planned=recent_planned[-1] if recent_planned else None,
        odom_samples_in_previous_second=sum(t-1_000_000_000<=s['receipt_ns']<=t for s in samples),
        source_age_at_event_s=(t-last['source_ns'])/1e9 if last else None,
        latest_source_admitted_at_event=last['admitted_at_event'] if last else None,
        last_odom_receipt_age_s=(t-last['receipt_ns'])/1e9 if last else None,
        max_receipt_gap_near_event_s=max(((b['receipt_ns']-a['receipt_ns'])/1e9 for a,b in zip(samples,samples[1:])),default=None),
        samples=samples))
db.close()
print(json.dumps(dict(trial=str(trial),events=result),indent=2))
