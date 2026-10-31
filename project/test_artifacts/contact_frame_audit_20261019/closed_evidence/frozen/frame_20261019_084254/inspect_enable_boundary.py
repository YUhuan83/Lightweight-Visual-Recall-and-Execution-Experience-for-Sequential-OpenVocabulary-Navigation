"""Read a closed diagnostic's enable/ACK boundary without assuming callback order."""
from pathlib import Path
import json
import sqlite3
import sys
from rclpy.serialization import deserialize_message
from std_msgs.msg import String

run=Path(sys.argv[1]);events=[json.loads(s) for s in (run/'contact_result.events.jsonl').read_text().splitlines()]
enabled=next(v['wall_ns'] for v in events if v['kind']=='speed_gate' and v['enabled'])
con=sqlite3.connect('file:'+str(next((run/'interfaces').glob('*.db3')))+'?mode=ro',uri=True)
topics={n:i for i,n in con.execute('SELECT id,name FROM topics')}
rows=[]
for topic in ('/wildos/control_status','/wildos/native_control_status'):
    for t,data in con.execute('SELECT timestamp,data FROM messages WHERE topic_id=? AND timestamp BETWEEN ? AND ? ORDER BY timestamp,id',(topics[topic],enabled-200_000_000,enabled+300_000_000)):
        value=json.loads(deserialize_message(data,String).data)
        if topic=='/wildos/control_status':
            owner=value.get('source_control') or {}
            row=dict(topic='control',reason=value['reason'],enabled=value['enabled'],source_ack_ready=value['source_ack_ready'],generation=owner.get('generation'),sequence=owner.get('sequence'),native_source_wall_time=value.get('native_source_wall_time'))
        else:
            owner=value.get('source_receipt') or {}
            row=dict(topic='native',generation=owner.get('generation'),sequence=owner.get('sequence'),applied=owner.get('applied'),wall_time=value['wall_time'],sent=value['sent'])
        row['relative_bag_ms']=(t-enabled)/1e6;rows.append(row)
con.close();rows.sort(key=lambda r:r['relative_bag_ms'])
report=dict(scope=__doc__,enable_wall_ns=enabled,events=events,observed_bag_order=rows,
    actual_probe_callback_order_recorded=False,conclusion='Recorded topic order is evidence of the generation transition, not exact probe-cache state.')
(run/'enable_boundary.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
