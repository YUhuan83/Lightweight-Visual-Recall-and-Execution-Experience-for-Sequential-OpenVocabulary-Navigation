import json,sqlite3
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
root=Path('/home/ly/execution_continuity_review_20261022/v37_full_path')
out=[]
for trial in sorted((root/'full').glob('trial_*')):
    assert json.loads((trial/'result.json').read_text())['closed']
    goal=next(trial.glob('sessions/*/goal_001'));db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
    topics={n:(i,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
    counts={}
    for name in ('/vehicles/ego/sensors/front_lidar','/spot1/ouster/front/points_filtered'):
        i,c=topics[name];values=[]
        for t,b in db.execute('select timestamp,data from messages where topic_id=? order by timestamp limit 6',(i,)):
            m=deserialize_message(b,c);values.append(dict(points=m.height*m.width,frame=m.header.frame_id,stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec))
        counts[name]=values
    out.append(dict(trial=trial.name,counts=counts));db.close()
print(json.dumps(out,indent=2))
