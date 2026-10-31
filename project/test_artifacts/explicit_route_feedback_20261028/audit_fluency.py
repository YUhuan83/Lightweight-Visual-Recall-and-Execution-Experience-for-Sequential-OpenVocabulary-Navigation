"""Closed bags, identical odometry receipt-time fluency metrics."""
import json,math,sqlite3,sys,collections
from pathlib import Path
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
trial=Path(sys.argv[1]);assert json.loads((trial/'result.json').read_text())['closed']
goal=next(trial.glob('sessions/*/goal_001'));ev=[json.loads(x) for x in (goal/'events.jsonl').read_text().splitlines()]
q=next(x['elapsed'] for x in ev if x.get('kind')=='query_sent');f=next(x for x in ev if x.get('stamp_ns'))
origin=f['stamp_ns']-round((f['elapsed']-q)*1e9)
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
i=db.execute('select id from topics where name=?',('/spot1/odom',)).fetchone()[0]
rows=[(t,deserialize_message(b,Odometry)) for t,b in db.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp',(i,origin,origin+180_000_000_000))];db.close()
stop=run=longest=total=0.;last_sign=0;reversals=0;gaps=[]
for (t,a),(u,b) in zip(rows,rows[1:]):
 dt=(u-t)*1e-9;total+=dt
 if dt>.5:gaps.append(dt)
 speed=math.hypot(a.twist.twist.linear.x,a.twist.twist.linear.y)
 if speed<.1:stop+=dt;run+=dt;longest=max(longest,run)
 else:
  run=0.;sign=1 if a.twist.twist.linear.x>0 else -1
  if last_sign and sign!=last_sign:reversals+=1
  last_sign=sign
points=[m.pose.pose.position for _,m in rows]
states=[x for x in ev if x.get('kind')=='plan_status' and q<=x['elapsed']<=q+180]
durations=collections.defaultdict(float)
for a,b in zip(states,states[1:]):durations[a['status']]+=b['elapsed']-a['elapsed']
out=dict(scope='180s receipt-time cropped odom; speed <0.1m/s near-stop; measured direction changes are not gearbox changes; status duration is not stopped duration',trial=str(trial),coverage_s=total,near_stop_percent=100*stop/total if total else None,longest_near_stop_s=longest,measured_motion_direction_changes=reversals,odom_gaps_over_half_second=gaps,net_m=math.hypot(points[-1].x-points[0].x,points[-1].y-points[0].y),distance_m=sum(math.hypot(b.x-a.x,b.y-a.y) for a,b in zip(points,points[1:])),status_seconds=dict(durations),planning_calls=sum(x['status']=='planning' for x in states),background_calls=sum(x.get('status')=='planning' and x.get('background') is True for x in states))
(trial/'fluency_odom_audit.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
