"""Inspect frozen full paths inside long handoffs using the CLOSED sensor bag."""
from pathlib import Path
import json,sqlite3,math,re
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Path as RosPath
from wildos_nav2_adapter.motion_path import motion_sections

root=Path('/home/ly/motion_handoff_field_20261018')
h=json.loads((root/'closed_handoff_review.json').read_text())
s=Path(h['session']);g=s/'goal_001'
assert (s/'session_result.json').is_file() and (g/'result.json').is_file()
driver=sorted(root.glob('field_yaw_left90_search_*'))[-1]
assert (driver/'result.json').is_file()
stack=(s/'stack.log').read_text().splitlines()
paths=[]
with sqlite3.connect(next((s/'sensor_interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True) as c:
    topic=c.execute("select id from topics where name='/wildos/full_planned_path'").fetchone()[0]
    for stamp,data in c.execute('select timestamp,data from messages where topic_id=? order by timestamp',(topic,)):
        msg=deserialize_message(data,RosPath)
        points=[(p.pose.position.x,p.pose.position.y) for p in msg.poses]
        if len(points)<2:continue
        segments=motion_sections(points);first,last=segments[0]
        q=msg.poses[first].pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        edge=(points[first+1][0]-points[first][0],points[first+1][1]-points[first][1])
        paths.append(dict(received_ns=stamp,length=sum(math.dist(a,b) for a,b in zip(points,points[1:])),
            sections=len(segments),first_forward=edge[0]*math.cos(yaw)+edge[1]*math.sin(yaw)>0,
            first_length=sum(math.dist(a,b) for a,b in zip(points[first:last],points[first+1:last+1])),
            first_endpoint=points[last],endpoint=points[-1]))
results=[]
for lease in h['handoff_leases']:
    if not lease['long_section']:continue
    lo=lease['start']['stamp_ns'];hi=lease['outcome']['stamp_ns']
    lines=[l for l in stack if (m:=re.search(r'\[(\d+\.\d+)\]',l)) and lo/1e9<=float(m[1])<=hi/1e9]
    results.append(dict(start_elapsed=lease['start']['elapsed'],
        full_paths=[p for p in paths if lo<=p['received_ns']<=hi],
        requests=[l for l in lines if 'ComputePath request sent:' in l],
        results=[l for l in lines if 'ComputePath result:' in l],
        extension_rejections=[l for l in lines if 'extension_rejected' in l],
        background_retirements=[l for l in lines if '"status": "exploration_handoff_waiting"' in l]))
(root/'closed_extension_geometry.json').write_text(json.dumps(results,indent=2)+'\n')
print(json.dumps(results,indent=2))
