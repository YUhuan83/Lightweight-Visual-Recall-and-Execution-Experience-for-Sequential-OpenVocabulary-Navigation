from pathlib import Path
import json,sqlite3,bisect,collections
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
root=Path('/home/ly/cusp_execution_repair_20261016'); root.mkdir(exist_ok=True)
session=Path('/home/ly/frontier_motion_handoff_20261016/field_results/session_20261016_205443')
assert (session/'session_result.json').is_file() and (session/'goal_001/result.json').is_file()
assert Path('/home/ly/frontier_motion_handoff_20261016/field_scope_baseline_search_20261016_205441/result.json').is_file()
db=sqlite3.connect(next((session/'goal_001/interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
def records(topic):
    tid,typ=db.execute('select id,type from topics where name=?',(topic,)).fetchone()
    cls=get_message(typ)
    return [(t,deserialize_message(b,cls)) for t,b in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid,))]
controls=records('/wildos/native_control')
native=[(t,json.loads(m.data)) for t,m in records('/wildos/native_control_status')]
status=[(t,json.loads(m.data)) for t,m in records('/wildos/control_status')]
report=dict(message_fields=controls[0][1].get_fields_and_field_types(),windows=[])
for at in (1792155514.481473078,1792155518.831062212,1792155525.274505012):
    report['windows'].append(dict(failure_wall=at,
        controls=[dict(received_ns=t,**{k:getattr(m,k) for k in ('throttle','brake','steering','gear')}) for t,m in controls if at-1.5<t/1e9<at+.3],
        status=[dict(received_ns=t,**m) for t,m in status if at-1.5<t/1e9<at+.3],
        native=[dict(received_ns=t,**m) for t,m in native if at-1.5<t/1e9<at+.3]))
(root/'delivery_review.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report['message_fields']))
for w in report['windows']:
    a=next((m for m in w['status'] if m['brake']>.2),None)
    b=next((m for m in w['controls'] if m['brake']>.2),None)
    c=next((m for m in w['native'] if m['sent']['brake']>.2),None)
    print('failure',w['failure_wall'],'brake status',a['stamp_ns']/1e9 if a else None,
        'control received',b['received_ns']/1e9 if b else None,'sent native',c['wall_time'] if c else None)
