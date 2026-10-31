"""Inspect CLOSED recordings only: trace localized target identity and acquisition time."""
import collections
import json
import math
from pathlib import Path
import sqlite3
import sys
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

trial = Path(sys.argv[1])
assert json.loads((trial/'result.json').read_text())['closed']
goal = next(trial.glob('sessions/*/goal_001'))
events = [json.loads(x) for x in (goal/'events.jsonl').read_text().splitlines()]
query = next(x['elapsed'] for x in events if x.get('kind') == 'query_sent')
first = next(x for x in events if x.get('stamp_ns'))
origin = first['stamp_ns']-round((first['elapsed']-query)*1e9)
db = sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro', uri=True)
out = {'trial': str(trial), 'topics': {}, 'flows': {}}
for topic_id,name,type_name in db.execute('select id,name,type from topics').fetchall():
    if not any(word in name for word in ('object','mask','bearing','destination','waypoint')):
        continue
    rows = db.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp',
                      (topic_id,origin,origin+120000000000)).fetchall()
    out['topics'][name] = len(rows)
    cls = get_message(type_name)
    flow = []
    for timestamp,data in rows:
        msg = deserialize_message(data,cls)
        item = {'q': round((timestamp-origin)*1e-9,3)}
        if type_name == 'std_msgs/msg/String':
            try: value=json.loads(msg.data)
            except (ValueError,TypeError): continue
            for key in ('source','position','object_position','localization_valid','search_confirmed',
                        'reason','evidence_rejected','instance_id','instance_status','approach_refresh'):
                if key in value: item[key]=value[key]
            quality=value.get('quality',{})
            item['quality']={key:quality[key] for key in ('points','confidence','representative_kind','association_status') if key in quality}
            stamp=value.get('observation_stamp_ns')
            if stamp: item['observation_q']=round((stamp-origin)*1e-9,3)
        elif type_name == 'geometry_msgs/msg/PoseStamped':
            p=msg.pose.position;item['position']=[p.x,p.y,p.z]
            item['stamp_q']=round((msg.header.stamp.sec*10**9+msg.header.stamp.nanosec-origin)*1e-9,3)
        else:
            if hasattr(msg,'header'):
                item['stamp_q']=round((msg.header.stamp.sec*10**9+msg.header.stamp.nanosec-origin)*1e-9,3)
        flow.append(item)
    out['flows'][name]=flow
db.close()
(trial/'target_flow_audit.json').write_text(json.dumps(out,indent=2))
print(json.dumps(out,indent=2))
