"""Read only CLOSED bags; compare recorded frontier offers at one fixed pose/map."""
from pathlib import Path
import json
import math
import sqlite3
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

root=Path('/home/ly/exploration_efficiency_audit_20261016')
session=Path('/home/ly/wildos-console/audit_results/session_20261016_095740')
assert (session/'session_result.json').is_file() and (session/'goal_001/result.json').is_file()
assert Path('/home/ly/wildos-console/field_exploration_owner_search_20261016_095739/result.json').is_file()
case=json.loads((root/'cases.json').read_text())['cases'][0]
db=sqlite3.connect(next((session/'sensor_interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
def closest(name):
    topic,kind=db.execute('select id,type from topics where name=?',(name,)).fetchone()
    stamp,data=db.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',(topic,case['stamp'])).fetchone()
    return stamp,deserialize_message(data,get_message(kind))
received,graph=closest('/spot1/scored_nav_graph')
ci=graph.trav_classes.index('default')
topic,kind=db.execute('select id,type from topics where name=?',('/spot1/nav_graph',)).fetchone()
raw=None
for stamp,data in db.execute('select timestamp,data from messages where topic_id=? and timestamp<=? and timestamp>=? order by timestamp desc',(topic,case['stamp'],received-5*10**9)):
    candidate=deserialize_message(data,get_message(kind))
    if (candidate.header==graph.header and candidate.trav_classes==graph.trav_classes
            and candidate.current_node_idx==graph.current_node_idx
            and len(candidate.nodes)==len(graph.nodes)
            and all(a.uuid==b.uuid and a.pose==b.pose for a,b in zip(candidate.nodes,graph.nodes))):
        raw=candidate
        break
assert raw is not None,'No exact topology snapshot'
graph_time=graph.header.stamp.sec+graph.header.stamp.nanosec*1e-9
def assessment(items):
    a=next((x for x in items if x.robot_profile_id=='default'),None)
    if a is None or a.observed_at.sec<=0 or not math.isfinite(a.confidence) or not .7<=a.confidence<=1.:
        return None
    age=graph_time-a.observed_at.sec-a.observed_at.nanosec*1e-9
    return a if 0.<=age<=5. else None
adj={i:set() for i in range(len(graph.nodes))}
for edge in raw.edges:
    a,b=edge.from_idx,edge.to_idx
    if a not in adj or b not in adj or ci>=len(edge.traversability):
        continue
    trav=edge.traversability[ci]
    if not math.isfinite(trav.traversability_cost) or trav.traversability_cost<0:
        continue
    traversed=any(p.key=='traversed' and p.value and p.value[0]>.5 for p in trav.properties)
    endpoints=all((v:=assessment(graph.nodes[i].robot_traversability)) is None or v.feasible for i in (a,b))
    v=assessment(edge.robot_traversability)
    if traversed or (endpoints and not (v is not None and v.physical_traversability<=0)):
        adj[a].add(b);adj[b].add(a)
reachable={graph.current_node_idx};work=list(reachable)
while work:
    for i in adj[work.pop()]-reachable:
        reachable.add(i);work.append(i)
rows=[]
for i,node in enumerate(graph.nodes):
    if ci>=len(node.trav_properties) or not node.trav_properties[ci].is_frontier:
        continue
    p=node.pose.position
    distance=math.hypot(p.x-case['start'][0],p.y-case['start'][1])
    if distance<=3.:
        continue
    headings=[h for h in node.heading_candidates if h.traversable and math.isfinite(h.heading) and math.isfinite(h.predicted_utility)]
    if not headings:
        continue
    h=max(headings,key=lambda h:h.predicted_utility)
    assessments=[dict(profile=a.robot_profile_id,confidence=a.confidence,feasible=a.feasible,
        physical=a.physical_traversability,stamp=a.observed_at.sec+a.observed_at.nanosec*1e-9)
        for a in node.robot_traversability]
    rows.append(dict(node=i,graph_reachable=i in reachable,uuid=bytes(node.uuid.id).hex(),goal=[p.x,p.y,h.heading],
        semantic_utility=h.predicted_utility,distance_m=distance,
        heading_motion_cost=h.motion_cost,assessments=assessments))
rows.sort(key=lambda r:r['semantic_utility']-.004*r['distance_m'],reverse=True)
out=root/'frontier_alternatives';out.mkdir(exist_ok=True)
map_text=(root/'input0.txt').read_text();lines=map_text.splitlines();header=lines[0].split()
for index,row in enumerate(rows[:8]):
    row['index']=index+20
    (out/f'input{index+20}.txt').write_text(' '.join(header[:8]+list(map(str,row['goal'])))+'\n'+lines[1]+'\n')
report=dict(scope='Closest scored snapshot before first full path. Same published map/actual path start. Exact topology match and initial graph connectivity (default profile, no rejection overlay). Diagnostic candidates only; not the earlier original selection snapshot or private costmap equivalence.',
    received_ns=received,graph_stamp_ns=graph.header.stamp.sec*10**9+graph.header.stamp.nanosec,
    start=case['start'],candidates=rows[:8])
(out/'candidates.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
