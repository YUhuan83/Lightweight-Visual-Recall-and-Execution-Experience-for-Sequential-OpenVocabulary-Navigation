"""Bounded closed public-map replay inputs; not exact private costmap snapshots."""
import hashlib,json,math,re,sqlite3
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

root=Path('/home/ly/planning_phase_physical_20261018')
session=root/'field_results/session_20261018_151920';goal=session/'goal_001'
out=Path('/home/ly/vehicle_interface_audit_20261018');out.mkdir(exist_ok=True)
assert (session/'session_result.json').is_file() and (goal/'result.json').is_file()
events=[json.loads(l) for l in (goal/'events.jsonl').read_text().splitlines()]
failure=next(e for e in events if e.get('status')=='no_vehicle_path')
requests=[]
for line in (session/'stack.log').read_text().splitlines():
    t=re.search(r'\[(\d+\.\d+)\]',line)
    m=re.search(r'ComputePath request sent: epoch=(\d+) candidate=(\d+) request=(\d+) goal=\(([-\d.]+), ([-\d.]+)\)',line)
    if t and m:requests.append(dict(wall=float(t[1]),epoch=int(m[1]),candidate=int(m[2]),serial=int(m[3]),x=float(m[4]),y=float(m[5]),line=line))
db=next((goal/'interfaces').glob('*.db3'))
c=sqlite3.connect(db.as_uri()+'?mode=ro',uri=True)
topics={name:(i,typ) for i,name,typ in c.execute('select id,name,type from topics')}
def before(topic,ns):
    i,typ=topics[topic]
    row=c.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',(i,ns)).fetchone()
    assert row,topic
    return row[0],deserialize_message(row[1],get_message(typ))
report=dict(scope=__doc__,failure=failure,cases=[])
for index in (0,6,7):
    target=failure['candidate_goals'][index]
    matches=[r for r in requests if r['candidate']==index and abs(r['x']-target[0])<.006 and abs(r['y']-target[1])<.006 and r['wall']<failure['stamp_ns']/1e9]
    assert matches,(index,target)
    request=matches[-1];ns=round(request['wall']*1e9)
    ot,odom=before('/spot1/odom',ns);mt,grid=before('/global_costmap/costmap',ns)
    p=odom.pose.pose;q=p.orientation
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    info=grid.info
    path=out/f'case{index}.txt'
    lines=[' '.join(map(str,(info.width,info.height,info.resolution,info.origin.position.x,info.origin.position.y,p.position.x,p.position.y,yaw,*target)))]
    lines.extend(f'{i} {v}' for i,v in enumerate(grid.data) if v!=0)
    path.write_text('\n'.join(lines)+'\n')
    report['cases'].append(dict(index=index,input=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),request=request,
        start=[p.position.x,p.position.y,yaw],goal=target,
        odom_received_age_sec=(ns-ot)/1e9,odom_source_age_sec=(ns-(odom.header.stamp.sec*10**9+odom.header.stamp.nanosec))/1e9,
        map_received_age_sec=(ns-mt)/1e9,map_source_age_sec=(ns-(grid.header.stamp.sec*10**9+grid.header.stamp.nanosec))/1e9,
        resolution=info.resolution,frame=grid.header.frame_id,nonzero_cells=len(lines)-1,
        reconstruction='Lethal/unknown cells plus fresh inflation using unchanged physical parameters; public occupied-probability grid is not exact private raw cost'))
c.close();(out/'planner_cases.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report['cases'],indent=2))
