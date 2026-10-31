import json,sqlite3,math
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from graphnav_msgs.msg import NavigationGraph
base=Path('/home/ly/execution_continuity_review_20261022')
root=base/'frontier_endpoint_contract_review'
line=next(s for s in (root/'recorded_graphs.txt').read_text().splitlines() if 'pair_000156_' in s)
raw,scored,gx,gy,gz,stamp=line.split();stamp=int(stamp)
receipt=json.loads((base/'v54_confirmed_graph_handoff/field_result.json').read_text())
assert receipt['result']['closed']
goal=next(Path(receipt['trial']).glob('sessions/*/goal_001'))
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
def last(name):
    i,c=topics[name];t,data=db.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',(i,stamp)).fetchone()
    return t,deserialize_message(data,c)
ot,o=last('/spot1/odom');mt,m=last('/global_costmap/costmap_raw');db.close()
p=o.pose.pose.position;q=o.pose.pose.orientation
yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
g=deserialize_message(Path(scored).read_bytes(),NavigationGraph)
targets=[]
for index in (19,53):
    n=g.nodes[index];x,y=n.pose.position.x,n.pose.position.y
    heading=math.atan2(float(gy)-y,float(gx)-x)
    targets.extend([[x,y,heading],[x,y,yaw]])
    points=n.trav_properties[0].frontier_points
    if points:
        f=min(points,key=lambda f:math.hypot(f.x-float(gx),f.y-float(gy)))
        targets.append([f.x,f.y,heading])
meta=m.metadata
out=root/'forward_smac.txt'
with out.open('w') as f:
    f.write(' '.join(map(str,[meta.size_x,meta.size_y,meta.resolution,meta.origin.position.x,meta.origin.position.y,p.x,p.y,yaw,len(targets)]))+'\n')
    for target in targets:f.write(' '.join(map(str,target))+'\n')
    for index,cost in enumerate(m.data):
        if cost:f.write(f'{index} {cost}\n')
(root/'forward_smac_scope.json').write_text(json.dumps(dict(graph_stamp_ns=stamp,odom_receipt_ns=ot,map_receipt_ns=mt,odom_age_sec=(stamp-ot)*1e-9,map_age_sec=(stamp-mt)*1e-9,start=[p.x,p.y,yaw],targets=targets,scope='Actual recorded raw global costmap and odometry; genuine 12m Smac; no action or vehicle counterfactual'),indent=2))
print(out)
