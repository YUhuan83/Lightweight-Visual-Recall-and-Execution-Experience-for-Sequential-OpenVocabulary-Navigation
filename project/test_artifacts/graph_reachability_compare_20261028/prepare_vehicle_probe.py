import json,sqlite3,math
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from graphnav_msgs.msg import NavigationGraph
base=Path('/home/ly/execution_continuity_review_20261022/graph_reachability_compare_20261028')
trial=Path('/home/ly/execution_continuity_review_20261022/v62_pending_route_owner/full/trial_20261028_222746')
goal=next(trial.glob('sessions/*/goal_001'));assert json.loads((trial/'result.json').read_text())['closed']
raw,scored,gx,gy,gz,stamp=next(x for x in (base/'inputs.txt').read_text().splitlines() if 'pair_000317_' in x).split();stamp=int(stamp)
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
def last(n):
 i,c=topics[n];t,b=db.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',(i,stamp)).fetchone();return t,deserialize_message(b,c)
ot,o=last('/spot1/odom');mt,m=last('/global_costmap/costmap_raw');db.close()
p=o.pose.pose.position;q=o.pose.pose.orientation
yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
g=deserialize_message(Path(scored).read_bytes(),NavigationGraph);node=g.nodes[7].pose.position
targets=[];labels=[]
for label,(x,y) in [('node7',(node.x,node.y)),('selected_frontier',(734.7486032884394,-82.66180447487591))]:
 bearing=math.atan2(y-p.y,x-p.x)
 for kind,h in [('target',math.atan2(float(gy)-y,float(gx)-x)),('current',yaw),('arc',math.remainder(2*bearing-yaw,2*math.pi))]:
  targets.append([x,y,h]);labels.append([label,kind])
meta=m.metadata
with (base/'vehicle_probe.txt').open('w') as f:
 f.write(' '.join(map(str,[meta.size_x,meta.size_y,meta.resolution,meta.origin.position.x,meta.origin.position.y,p.x,p.y,yaw,len(targets)]))+'\n')
 for t in targets:f.write(' '.join(map(str,t))+'\n')
 for i,c in enumerate(m.data):
  if c:f.write(f'{i} {c}\n')
(base/'vehicle_probe_scope.json').write_text(json.dumps(dict(graph_stamp_ns=stamp,odom_receipt_age_s=(stamp-ot)*1e-9,map_receipt_age_s=(stamp-mt)*1e-9,start=[p.x,p.y,yaw],targets=targets,labels=labels,scope='recorded map, real planner probe, no vehicle counterfactual'),indent=2))
print((base/'vehicle_probe_scope.json').read_text())
