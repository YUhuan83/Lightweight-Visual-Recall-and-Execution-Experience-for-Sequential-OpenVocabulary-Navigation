from pathlib import Path
import json,sqlite3,math
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
s=Path('/home/ly/wildos-console/audit_results/session_20261016_095740');r=Path('/home/ly/exploration_efficiency_audit_20261016');r.mkdir(exist_ok=True)
assert (s/'session_result.json').exists() and (s/'goal_001/result.json').exists()
c=sqlite3.connect(next((s/'goal_001/interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
raw=sqlite3.connect(next((s/'sensor_interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
def topic(db,name):
 i,t=db.execute('select id,type from topics where name=?',(name,)).fetchone();return i,get_message(t)
def closest(db,name,stamp):
 i,t=topic(db,name);row=db.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',(i,stamp)).fetchone();assert row,name
 return row[0],deserialize_message(row[1],t)
tid,kind=topic(raw,'/wildos/full_planned_path');row=raw.execute('select timestamp,data from messages where topic_id=? order by timestamp limit 1',(tid,)).fetchone();first_ts,path=row[0],deserialize_message(row[1],kind)
def yaw(p):
 q=p.orientation;return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
_,meta=closest(c,'/spot1/graphnav_planner/route_status',first_ts);meta=json.loads(meta.data)
start=[path.poses[0].pose.position.x,path.poses[0].pose.position.y,yaw(path.poses[0].pose)]
goal=[*meta['goal'],meta['observation_heading']];bearing=math.atan2(goal[1]-start[1],goal[0]-start[0])
cases=[dict(label='initial_observation_heading',stamp=first_ts,start=start,goal=goal),dict(label='initial_approach_bearing',stamp=first_ts,start=start,goal=[*goal[:2],bearing]),dict(label='initial_start_heading',stamp=first_ts,start=start,goal=[*goal[:2],start[2]])]
events=[json.loads(l) for l in (s/'goal_001/events.jsonl').read_text().splitlines()]
failed=next(x for x in events if x.get('status')=='no_vehicle_path');stamp=failed['stamp_ns'];_,odom=closest(c,'/spot1/odom',stamp)
point=odom.pose.pose;start=[point.position.x,point.position.y,yaw(point)];goal=failed['candidate_goals'][0];bearing=math.atan2(goal[1]-start[1],goal[0]-start[0])
cases.extend([dict(label='stopped_original_heading',stamp=stamp,start=start,goal=goal),dict(label='stopped_approach_bearing',stamp=stamp,start=start,goal=[*goal[:2],bearing]),dict(label='stopped_start_heading',stamp=stamp,start=start,goal=[*goal[:2],start[2]])])
for length in (3.,-3.,6.,-6.):
 cases.append(dict(label='stopped_straight_'+str(length),stamp=stamp,start=start,goal=[start[0]+length*math.cos(start[2]),start[1]+length*math.sin(start[2]),start[2]]))
for i,case in enumerate(cases):
 ts,grid=closest(c,'/global_costmap/costmap',case['stamp']);case.update(index=i,map_received_ns=ts,map_stamp_ns=grid.header.stamp.sec*10**9+grid.header.stamp.nanosec)
 header=[grid.info.width,grid.info.height,grid.info.resolution,grid.info.origin.position.x,grid.info.origin.position.y,*case['start'],*case['goal']]
 (r/('input'+str(i)+'.txt')).write_text(' '.join(map(str,header))+'\n'+' '.join(map(str,grid.data))+'\n')
(r/'cases.json').write_text(json.dumps(dict(scope='Closed recorded output start + inferred initial request or last stopped odometry; closest prior published OccupancyGrid. Reconstructed cost quantization, not exact private planner state/action request.',cases=cases),indent=2));print(json.dumps(cases,indent=2))
