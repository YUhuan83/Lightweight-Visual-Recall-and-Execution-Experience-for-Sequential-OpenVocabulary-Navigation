import json,sqlite3
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
trial=Path('/home/ly/execution_continuity_review_20261022/v60_replan_horizon/full/trial_20261028_215532')
assert json.loads((trial/'result.json').read_text())['closed']
goal=next(trial.glob('sessions/*/goal_001'));t=1793195885879874792
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
out=[]
for i,n,k in db.execute('select id,name,type from topics'):
 if n not in ('/spot1/raw_object_waypoint','/spot1/object_confirmation'):continue
 for stamp,b in db.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ? order by timestamp',(i,t-500000000,t+500000000)):
  m=deserialize_message(b,get_message(k))
  payload=json.loads(m.data) if hasattr(m,'data') else dict(x=m.pose.position.x,y=m.pose.position.y,z=m.pose.position.z)
  out.append(dict(topic=n,relative_s=(stamp-t)*1e-9,payload=payload))
db.close();(trial/'takeover_input_audit.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
