"""Actual map-cell obstacle/unknown distinction along a rejected forward edge."""
import json,sqlite3,math
from pathlib import Path
import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from graphnav_msgs.msg import NavigationGraph
base=Path('/home/ly/execution_continuity_review_20261022')
root=base/'frontier_endpoint_contract_review'
row=next(s for s in (root/'recorded_graphs.txt').read_text().splitlines() if 'pair_000156_' in s)
raw,scored,*_=row.split();g=deserialize_message(Path(raw).read_bytes(),NavigationGraph)
edge=next(e for e in g.edges if e.from_idx==68 and e.to_idx==337)
stamp=edge.robot_traversability[0].observed_at.sec*10**9+edge.robot_traversability[0].observed_at.nanosec
receipt=json.loads((base/'v54_confirmed_graph_handoff/field_result.json').read_text())
assert receipt['result']['closed']
goal=next(Path(receipt['trial']).glob('sessions/*/goal_001'))
db=sqlite3.connect(next((goal/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topics={n:(i,get_message(t)) for i,n,t in db.execute('select id,name,type from topics')}
def messages(name):
    i,c=topics[name]
    for b, in db.execute('select data from messages where topic_id=? order by timestamp',(i,)):
        yield deserialize_message(b,c)
def ns(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
m=next(m for m in messages('/spot1/local_terrain') if ns(m)==stamp)
tf=next(t for msg in messages('/tf') for t in msg.transforms if ns(t)==stamp and t.child_frame_id=='spot1/terrain_base_link')
assert m.header.frame_id==tf.child_frame_id and tf.header.frame_id=='spot1/odom'
db.close()
layers={n:np.asarray(d.data,dtype=np.float32).reshape((80,80))[::-1,::-1] for n,d in zip(m.layers,m.data)}
trav=layers['traversability'];unknown=layers['unknown']
conservative=cv2.distanceTransform(np.uint8(np.isfinite(trav)&(trav<=.99)),cv2.DIST_L2,cv2.DIST_MASK_5)*.5
known=cv2.distanceTransform(np.uint8(~(np.isfinite(trav)&(trav>.99))),cv2.DIST_L2,cv2.DIST_MASK_5)*.5
t=tf.transform.translation;q=tf.transform.rotation;heading=2*math.atan2(q.z,q.w)
rotation=np.array([[math.cos(heading),-math.sin(heading)],[math.sin(heading),math.cos(heading)]])
def cell(index):
    p=g.nodes[index].pose.position
    local=(np.array([p.x,p.y])-[t.x,t.y])@rotation
    return np.floor((local+20)/.5).astype(int)
a,b=cell(68),cell(337);line=np.zeros((80,80),np.uint8)
assert np.all(a>=0)&np.all(a<80)&np.all(b>=0)&np.all(b<80)
cv2.line(line,tuple(a),tuple(b),1,1,cv2.LINE_8)
ys,xs=np.where(line);i=np.argmin(conservative[ys,xs]);x,y=xs[i],ys[i]
obstacles=np.argwhere(np.isfinite(trav)&(trav>.99));nearest=obstacles[np.argmin(np.linalg.norm(obstacles-[y,x],axis=1))]
report=dict(scope=__doc__,stamp_ns=stamp,edge=[68,337],map_frame=m.header.frame_id,
    recorded_clearance=next(p.value[0] for p in edge.properties if p.key=='minimum_clearance'),
    raster_note='Python OpenCV LINE_8; not a substitute for grid_map LineIterator exact traversal',
    conservative_min_m=float(conservative[ys,xs].min()),known_obstacle_min_m=float(known[ys,xs].min()),
    nonfinite_traversability_cells=int((~np.isfinite(trav)).sum()),unknown_cells=int((unknown>.5).sum()),
    unknown_cells_marked_obstacle=int(((unknown>.5)&(trav>.99)).sum()),
    nearest_obstacle_cell=nearest.tolist(),nearest_obstacle_traversability=float(trav[tuple(nearest)]),nearest_obstacle_unknown=float(unknown[tuple(nearest)]))
(root/'clearance_source.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
