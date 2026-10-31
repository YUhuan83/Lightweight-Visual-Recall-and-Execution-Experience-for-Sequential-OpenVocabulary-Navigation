from pathlib import Path
import json,sqlite3,sys
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
trial=Path(sys.argv[1]);assert json.loads((trial/'result.json').read_text())['closed']
goal=next(trial.glob('sessions/*/goal_001'))
events=[json.loads(s) for s in (goal/'events.jsonl').read_text().splitlines()]
failures=[e for e in events if e.get('status')=='execution_unavailable']
db=sqlite3.connect(next(goal.rglob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics=dict(db.execute('select name,id from topics'))
out=[]
for event in failures:
    item={'event':event,'samples':{}}
    for topic in ('/wildos/native_control_status','/wildos/control_status'):
        rows=[]
        for ts,data in db.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp',(topics[topic],event['stamp_ns']-180_000_000,event['stamp_ns']+180_000_000)):
            row=json.loads(deserialize_message(data,String).data)
            keep=('wall_time','stamp_ns','speed','control_acknowledged','actuator_ok','telemetry_ok',
                  'telemetry_age_sec','source_receipt','source_rejection','watchdog_stop',
                  'source_ack_ready','source_control','reason','actuating')
            rows.append(dict(delta_ms=(ts-event['stamp_ns'])/1e6,**{k:row[k] for k in keep if k in row}))
        item['samples'][topic]=rows
    out.append(item)
(trial/'source_event_audit.json').write_text(json.dumps(out,indent=2))
print(json.dumps(out,indent=2))
