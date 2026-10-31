"""Read only closed bags; export the target-to-execution window, without replay."""
import json
import math
import sqlite3
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict

root = Path('/home/ly/source_control_repair_20261017')
session = root/'field_results/session_20261017_151623'
assert (session/'session_result.json').is_file()
assert (session/'goal_001/result.json').is_file()
origin = 1792221435.6587815
lo, hi = origin+370, origin+430

def stamp(h):
    return h.stamp.sec+h.stamp.nanosec/1e9

def pose(p):
    q = p.orientation
    return [p.position.x, p.position.y,
            math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]

def read(db, names):
    with sqlite3.connect(db.as_uri()+'?mode=ro', uri=True) as c:
        topics = {name:(i,typ) for i,name,typ in c.execute('select id,name,type from topics')}
        result = {}
        for name in names:
            if name not in topics:
                result[name] = []
                continue
            i,typ = topics[name]
            kind = get_message(typ)
            values = []
            for ts,blob in c.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp',
                                     (i,int(lo*1e9),int(hi*1e9))):
                m = deserialize_message(blob,kind)
                value = dict(received_wall=ts/1e9, relative_sec=ts/1e9-origin)
                if hasattr(m,'poses'):
                    value.update(frame=m.header.frame_id, stamp=stamp(m.header),
                                 poses=[pose(p.pose) for p in m.poses])
                elif hasattr(m,'query'):
                    value.update(query=m.query, frame=m.header.frame_id, stamp=stamp(m.header),
                                 fields=list(m.get_fields_and_field_types()))
                else:
                    value['data'] = json.loads(m.data)
                values.append(value)
            result[name] = values
        return result

report = dict(origin_wall=origin, scope='Closed 20261017_151623 only; relative time is query-send, video offset <~1s not calibrated')
report['goal_bag'] = read(session/'goal_001/interfaces/interfaces_0.db3',
    ['/spot1/object_mask','/spot1/visible_target_approach',
     '/spot1/object_localization','/spot1/object_confirmation',
     '/spot1/graphnav_planner/route_status'])
report['sensor_bag'] = read(session/'sensor_interfaces/sensor_interfaces_0.db3',
    ['/wildos/full_planned_path'])
out = root/'visible_window.json'
out.write_text(json.dumps(report,indent=2))
print(json.dumps(dict(output=str(out),goal_counts={k:len(v) for k,v in report['goal_bag'].items()},
                     full_paths=len(report['sensor_bag']['/wildos/full_planned_path']))))
