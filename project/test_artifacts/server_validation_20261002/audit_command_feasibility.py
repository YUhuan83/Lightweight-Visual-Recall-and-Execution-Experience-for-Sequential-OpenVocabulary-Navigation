"""Measure vehicle command feasibility from the recorded test; no ROS mutations."""
import json,sys,sqlite3,subprocess,math
from pathlib import Path
sys.path.insert(0,'/home/ly')
from session_tools_20261002 import ROOT,WS,runtime_env
if '--runtime-child' not in sys.argv:
    raise SystemExit(subprocess.run([str(WS/'.venv/bin/python'),__file__,'--runtime-child'],env=runtime_env()).returncode)
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
db=sqlite3.connect('file:'+str(ROOT/'body_collision_full_trial1/rosbag/rosbag_0.db3')+'?mode=ro',uri=True)
tid,typ=db.execute("select id,type from topics where name='/wildos/cmd_vel'").fetchone();cls=get_message(typ)
report={'total':0,'nonzero':0,'command_radius_below_8m':0,'examples':[]}
for ts,blob in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid,)):
    m=deserialize_message(blob,cls);v=m.linear.x;w=m.angular.z;report['total']+=1
    if abs(v)<=.01:continue
    report['nonzero']+=1
    if abs(w/v)>1/8+1e-3:
        report['command_radius_below_8m']+=1
        if len(report['examples'])<5:report['examples'].append({'timestamp':ts,'v':v,'w':w,'radius':abs(v/w)})
db.close()
(ROOT/'mechanism_audit/command_feasibility.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report))

