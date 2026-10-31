import sqlite3,json,math
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
root=Path('/home/ly/wildos-console/results/session_20261015_151859');assert (root/'session_result.json').exists()
out=Path('/home/ly/search_mechanism_audit_20261015');out.mkdir(exist_ok=True)
c=sqlite3.connect(next((root/'goal_001/interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,get_message(t)) for i,n,t in c.execute('select id,name,type from topics')}
def previous(name,ns):
 i,t=topics[name];row=c.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',(i,ns)).fetchone();assert row
 return row[0],deserialize_message(row[1],t)
events=[json.loads(l) for l in (root/'goal_001/events.jsonl').read_text().splitlines()]
cases=[e for e in events if e.get('status')=='no_vehicle_path']
summary=[]
for k,e in enumerate((cases[0],cases[-1])):
 ns=e['stamp_ns'];ot,o=previous('/spot1/odom',ns);mt,m=previous('/global_costmap/costmap',ns)
 p=o.pose.pose;q=p.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
 candidate=e['candidate_goals'][0]
 values=[m.info.width,m.info.height,m.info.resolution,m.info.origin.position.x,m.info.origin.position.y,p.position.x,p.position.y,yaw,*candidate]
 (out/f'input{k}.txt').write_text(' '.join(map(str,values))+'\n'+' '.join(map(str,m.data)))
 summary.append(dict(case=k,status=e,odom_age=(ns-(o.header.stamp.sec*10**9+o.header.stamp.nanosec))/1e9,map_receipt_age=(ns-mt)/1e9,scope='closest prior published odometry and occupancy costmap; not exact private planner snapshot'))
(out/'inputs.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
