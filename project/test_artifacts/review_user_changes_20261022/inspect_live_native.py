import sqlite3,json,glob
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
p=glob.glob('/home/ly/execution_continuity_review_20261022/v3/full/trial_20261022_172808/sessions/session_*/goal_001/interfaces/*.db3')[0]
db=sqlite3.connect('file:'+p+'?mode=ro',uri=True)
name,typ,tid=db.execute("select name,type,id from topics where name='/wildos/native_control_status'").fetchone(); cls=get_message(typ)
ts0=1792661399942801630
rows=db.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp',(tid,ts0-1000000000,ts0+1000000000)).fetchall()
for ts,data in rows:
 v=json.loads(deserialize_message(data,cls).data)
 print(json.dumps({'bag_ns':ts,'wall_time':v.get('wall_time'),'telemetry_age_sec':v.get('telemetry_age_sec'),'telemetry_ok':v.get('telemetry_ok'),'actuator_ok':v.get('actuator_ok'),'control_acknowledged':v.get('control_acknowledged'),'speed':v.get('speed'),'gear':(v.get('actual') or {}).get('gear'),'brake':(v.get('actual') or {}).get('brake'),'throttle':(v.get('actual') or {}).get('throttle')},separators=(',',':')))
