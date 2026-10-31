from pathlib import Path
import bisect,collections,json,sqlite3,sys
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
trial=Path(sys.argv[1]);assert json.loads((trial/'result.json').read_text())['closed']
goal=next(trial.glob('sessions/*/goal_001'))
samples=[json.loads(s) for s in (goal/'native_samples.jsonl').read_text().splitlines()]
plans=[json.loads(s) for s in (goal/'events.jsonl').read_text().splitlines()]
plans=sorted((r['stamp_ns']/1e9,r) for r in plans if r['kind']=='plan_status')
db=sqlite3.connect(next(goal.rglob('*.db3')).as_uri()+'?mode=ro',uri=True)
tid=db.execute('select id from topics where name=?',('/wildos/control_status',)).fetchone()[0]
controls=[]
for timestamp,data in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid,)):
    row=json.loads(deserialize_message(data,String).data)
    controls.append((row.get('stamp_ns',timestamp)/1e9,row))
controls.sort(key=lambda r:r[0]);ct=[t for t,r in controls];pt=[t for t,r in plans]
reason=collections.Counter();pairs=collections.Counter();gear=collections.Counter()
for a,b in zip(samples,samples[1:]):
    dt=b['wall_time']-a['wall_time']
    if not 0<dt<=.5 or abs(a['speed'])>=.1:continue
    ci=bisect.bisect_right(ct,a['wall_time'])-1;pi=bisect.bisect_right(pt,a['wall_time'])-1
    c=controls[ci][1] if ci>=0 else {};p=plans[pi][1] if pi>=0 else {}
    r=c.get('reason','before_control');reason[r]+=dt
    pairs[p.get('status','before_plan')+'/'+r]+=dt
    if r=='gear_change':gear[str(c.get('gear_phase'))]+=dt
out=dict(scope='Closed recorded source-time join; speed<0.1m/s; valid dt<=0.5s',
         near_stop_by_control_reason_s=dict(reason),near_stop_by_plan_control_s=dict(pairs),gear_phases=dict(gear))
(trial/'control_stop_attribution.json').write_text(json.dumps(out,indent=2))
print(json.dumps(out,indent=2))
