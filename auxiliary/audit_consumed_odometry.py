"""Compare bridge-consumed odometry identities with admitted published samples."""
import hashlib
import json
import math
import sqlite3
import sys
from pathlib import Path
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from wildos_nav2_adapter.input_quality import stamp_ns,valid_pose,measurement_age

trial=Path(sys.argv[1])
assert json.loads((trial/'result.json').read_text())['closed']
goal=next(trial.glob('sessions/*/goal_001'))
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
tid=db.execute('select id from topics where name=?',('/spot1/odom',)).fetchone()[0]
odom=[(t,deserialize_message(blob,Odometry)) for t,blob in db.execute(
    'select timestamp,data from messages where topic_id=? order by timestamp',(tid,))]
db.close()
by_stamp={stamp_ns(message):message for _,message in odom}
plans={d['stamp_ns']:d for line in (goal/'events.jsonl').read_text().splitlines()
       for d in [json.loads(line)] if d['kind']=='plan_status' and d.get('replay_inputs')}
events=[]
for t,event in sorted(plans.items()):
    used=event['replay_inputs'].get('odom_stamp_ns')
    if not used:continue
    before=[(bt,m) for bt,m in odom if bt<=t and m.header.frame_id=='spot1/odom'
        and valid_pose(m.pose.pose) and measurement_age(m,t,1.,future_tolerance_sec=.05) is not None]
    if not before:continue
    bt,latest=max(before,key=lambda p:stamp_ns(p[1]))
    current=by_stamp.get(used)
    latest_stamp=stamp_ns(latest)
    delta=(math.dist((current.pose.pose.position.x,current.pose.pose.position.y),
                    (latest.pose.pose.position.x,latest.pose.pose.position.y)) if current else None)
    row=dict(elapsed=event['elapsed'],status=event['status'],reason=event.get('reason'),
        used_stamp_ns=used,used_age_s=(t-used)/1e9,latest_admitted_stamp_ns=latest_stamp,
        latest_source_age_s=(t-latest_stamp)/1e9,latest_bag_age_s=(t-bt)/1e9,
        source_lag_s=(latest_stamp-used)/1e9,position_difference_m=delta,
        replay_inputs=event['replay_inputs'],requested_goal=event.get('requested_goal_pose'),
        actual_start=event.get('actual_start_pose'))
    events.append(row)
output=dict(trial=str(trial),scope='Consumer source identity versus separately recorded admitted odometry; not direct DDS receipt tracing',
    examined=len(events),delayed=[e for e in events if e['source_lag_s']>=.15],
    cusp_refusals=[e for e in events if e['status']=='replanning_after_maneuver'])
encoded=json.dumps(output,sort_keys=True).encode()
(trial/'consumed_odometry_audit.json').write_bytes(encoded)
print(hashlib.sha256(encoded).hexdigest())
