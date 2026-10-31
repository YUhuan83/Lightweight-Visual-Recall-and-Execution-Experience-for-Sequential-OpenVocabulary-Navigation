"""Closed-bag evidence for visual intention changes and native lease exit."""
import bisect
import hashlib
import json
import math
import sqlite3
import sys
from pathlib import Path
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from std_msgs.msg import String

root=Path('/home/ly/execution_continuity_review_20261022/v19_dispatch_reuse')
all_output=[]
for trial in sorted((root/'full').glob('trial_*')):
    assert json.loads((trial/'result.json').read_text())['closed']
    goal=next(trial.glob('sessions/*/goal_001'))
    db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
    def read(topic,kind):
        tid=db.execute('select id from topics where name=?',(topic,)).fetchone()[0]
        return [(stamp,deserialize_message(blob,kind)) for stamp,blob in db.execute(
            'select timestamp,data from messages where topic_id=? order by timestamp',(tid,))]
    def strings(topic):
        return [(t,json.loads(m.data)) for t,m in read(topic,String)]
    unique_plans={d['stamp_ns']:d for _,d in strings('/wildos/plan_status')}
    plans=sorted(unique_plans.items())
    hints=strings('/spot1/visible_target_approach');odoms=read('/spot1/odom',Odometry)
    commands=[(m.header.stamp.sec*10**9+m.header.stamp.nanosec,m.twist.linear.x)
              for _,m in read('/wildos/cmd_vel_stamped',TwistStamped)]
    natives=sorted({d['wall_time']:(t,d) for t,d in strings('/wildos/native_control_status')}.values())
    db.close()
    events=[]
    for stamp,event in plans:
        if event['status']!='visible_target_approach' or event.get('continuous'):continue
        prior=[d for t,d in plans if t<stamp and d['status']=='planned']
        old=prior[-1] if prior else None
        odom=max(((t,m) for t,m in odoms if t<stamp),key=lambda x:x[0])[1]
        current=[odom.pose.pose.position.x,odom.pose.pose.position.y]
        previous_cmd=max(((t,v) for t,v in commands if t<stamp),default=(0,None))
        following_cmd=min(((t,v) for t,v in commands if t>stamp),default=(0,None))
        hint=max(((t,d) for t,d in hints if t<stamp+30_000_000),key=lambda x:x[0],default=(0,None))[1]
        old_goal=old.get('goal') if old else None
        point=event['goal']
        events.append(dict(event=event,current=current,previous_planned=old,hint=hint,
            old_endpoint_progress_to_new_goal_m=(math.dist(current,point)-math.dist(old_goal,point)
                                                 if old_goal else None),
            command_gap_s=(following_cmd[0]-previous_cmd[0])/1e9 if following_cmd[0] else None,
            previous_command=previous_cmd,next_command=following_cmd))
    unhealthy=[(t,d) for t,d in natives if d.get('actuator_ok') is False]
    native_summary=dict(last_keys=sorted(natives[-1][1]),last_native=natives[-1][1],
                        first_unhealthy=unhealthy[0] if unhealthy else None,
                        before_unhealthy=[(t,d) for t,d in natives
                            if unhealthy and unhealthy[0][0]-2_000_000_000<=t<=unhealthy[0][0]],
                        slow_samples=[(t,d) for t,d in natives if
                            (d.get('poll_roundtrip_ms') or 0)>200 or
                            (d.get('control_rpc_ms') or 0)>200 or (d.get('period') or 0)>.2],
                        before_exit=natives[-12:])
    output=dict(trial=str(trial),visual_handoffs=events,native_exit=native_summary,
                stale_odom_count=sum(d['status']=='stale_odom' for _,d in plans))
    (trial/'handoff_source_audit.json').write_text(json.dumps(output,indent=2))
    all_output.append(output)
encoded=json.dumps(all_output,sort_keys=True).encode()
print(hashlib.sha256(encoded).hexdigest())
