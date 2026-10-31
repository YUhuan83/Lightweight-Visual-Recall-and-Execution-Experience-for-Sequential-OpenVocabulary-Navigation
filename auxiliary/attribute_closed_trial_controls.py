"""Attribute recorded near-stop samples to contemporaneous control states."""
import bisect
import collections
import json
import sqlite3
import sys
from pathlib import Path
from rclpy.serialization import deserialize_message
from std_msgs.msg import String

trial=Path(sys.argv[1])
assert json.loads((trial/'result.json').read_text())['closed']
goal=next(trial.glob('sessions/*/goal_001'))
assert (goal/'stop.json').exists()
native=[json.loads(line) for line in (goal/'native_samples.jsonl').read_text().splitlines()]
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
def states(topic):
    tid=db.execute('select id from topics where name=?',(topic,)).fetchone()[0]
    output=[]
    for stamp,blob in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid,)):
        data=json.loads(deserialize_message(blob,String).data)
        output.append((data.get('stamp_ns',stamp)*1e-9,data))
    return sorted(output,key=lambda item:item[0])
control=states('/wildos/control_status');plans=states('/wildos/plan_status')
db.close()
ct=[t for t,_ in control];pt=[t for t,_ in plans]
by_control=collections.Counter();by_plan=collections.Counter();stale_by_plan=collections.Counter()
covered=near_stop=0.
for a,b in zip(native,native[1:]):
    t=a['wall_time'];dt=b['wall_time']-t
    if not 0<dt<=.5:continue
    covered+=dt
    if abs(a['speed'])>=.1:continue
    near_stop+=dt
    ci=bisect.bisect_right(ct,t)-1;pi=bisect.bisect_right(pt,t)-1
    reason=(control[ci][1].get('reason','unspecified')
            if ci>=0 and t-ct[ci]<=.5 else 'missing_recent_control_status')
    plan=plans[pi][1]['status'] if pi>=0 else 'before_plan_status'
    by_control[reason]+=dt;by_plan[plan]+=dt
    if reason=='stale_command':stale_by_plan[plan]+=dt
assert abs(sum(by_control.values())-near_stop)<1e-6
output=dict(native_coverage_s=covered,near_stop_s=near_stop,near_stop_pct=100*near_stop/covered,
            by_control_s=dict(by_control),by_plan_s=dict(by_plan),stale_by_plan_s=dict(stale_by_plan),
            method='Native intervals <=0.5s; speed <0.1m/s; last control <=0.5s old; state labels are attribution, not proof of root cause')
(trial/'control_stop_attribution.json').write_text(json.dumps(output,indent=2))
print(json.dumps(output,indent=2))
