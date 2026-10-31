"""Measure actual pose error before action aborts on recorded cusp segments."""
from pathlib import Path
import json
import sqlite3
import sys
import math
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.graph_path_bridge import xy,yaw
from wildos_nav2_adapter.graph_route import distance

root=Path('/home/ly/town_memory_validation_20261003')/sys.argv[1]
db=sqlite3.connect(f"file:{next((root/'rosbag').glob('*.db3'))}?mode=ro",uri=True)
topics={name:(i,get_message(kind)) for i,name,kind in db.execute('SELECT id,name,type FROM topics')}
def read(name):
    i,kind=topics[name]
    return [(t,deserialize_message(b,kind)) for t,b in db.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp',(i,))]
paths=read('/wildos/planned_path')
odoms=read('/spot1/odom')
commands=read('/wildos/cmd_vel')
result=[]
previous=None
for t,m in read('/wildos/plan_status'):
    d=json.loads(m.data)
    state=d.get('status')
    if state=='follow_failed' and previous!='follow_failed' and d.get('reason')=='follow_aborted':
        path=next(p for tp,p in reversed(paths) if tp<=t)
        end=path.poses[-1].pose
        recent=[m for to,m in odoms if t-5e9<to<=t]
        cmd=[m for tc,m in commands if t-5e9<tc<=t]
        errors=[distance(xy(m.pose.pose),xy(end)) for m in recent]
        headings=[abs(math.atan2(math.sin(yaw(m.pose.pose)-yaw(end)),math.cos(yaw(m.pose.pose)-yaw(end)))) for m in recent]
        result.append({'stamp':t,'goal':xy(end),'poses':len(path.poses),
            'pose_error_p50_min_max':np.percentile(errors,[50,0,100]).tolist(),
            'heading_error_deg_p50_min_max':np.rad2deg(np.percentile(headings,[50,0,100])).tolist(),
            'cmd_linear_min_max':[min(m.linear.x for m in cmd),max(m.linear.x for m in cmd)] if cmd else None,
            'actual_speed_max':max(math.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y) for m in recent)})
    previous=state
(root/'cusp_diagnosis.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
