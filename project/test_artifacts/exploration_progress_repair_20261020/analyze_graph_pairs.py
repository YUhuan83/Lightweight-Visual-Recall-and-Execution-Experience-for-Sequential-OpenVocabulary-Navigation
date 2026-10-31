"""Read sealed CDR snapshots; mirror physical edge admission, not the full stateful planner."""
from pathlib import Path
import argparse
import collections
import hashlib
import json
import math
from rclpy.serialization import deserialize_message
from graphnav_msgs.msg import NavigationGraph
from nav_msgs.msg import Odometry
from rosidl_runtime_py.convert import message_to_ordereddict

p=argparse.ArgumentParser();p.add_argument('root',type=Path);p.add_argument('--output',type=Path,required=True)
args=p.parse_args()

def ns(t):return t.sec*10**9+t.nanosec
def xy(n):return [n.pose.position.x,n.pose.position.y]
def prop(props,name):return next((list(v.value) for v in props if v.key==name),[])
def violation(n):return prop(n.properties,'physical_violation_observed')==[1.]
def assessment(n):return next((v for v in n.robot_traversability if v.robot_profile_id=='default'),None)
def fresh(a,now):
    return a is not None and a.observed_at.sec>0 and math.isfinite(a.confidence) and .7<=a.confidence<=1. and 0<=now-ns(a.observed_at)<=5e9
def plain(m):
    return json.loads(json.dumps(message_to_ordereddict(m),default=lambda a:a.tolist()))
def reach(adj,start):
    seen={start};pending=[start]
    while pending:
        for target in adj.get(pending.pop(),[]):
            if target not in seen:seen.add(target);pending.append(target)
    return seen

def load(item,typ):
    data=(args.root/item['file']).read_bytes()
    assert hashlib.sha256(data).hexdigest()==item['sha256']
    return deserialize_message(data,typ)

reports=[];errors=[]
for line in (args.root/'snapshots.jsonl').read_text().splitlines():
    row=json.loads(line)
    if row['kind']=='error':errors.append(row);continue
    if row['kind']!='pair' or not all(row['contract'].values()):continue
    g=load(row['raw'],NavigationGraph);scored=load(row['scored'],NavigationGraph)
    odom=load(row['odom'],Odometry) if 'odom' in row else None
    now=ns(g.header.stamp);ci=g.trav_classes.index('default');start=g.current_node_idx
    nodes=g.nodes;topo=collections.defaultdict(list);physical=collections.defaultdict(list)
    rejected=collections.Counter();incident=[]
    for e in g.edges:
        if ci>=len(e.traversability):continue
        f,t=e.from_idx,e.to_idx;topo[f].append(t)
        a=assessment(e);endpoints=[nodes[f],nodes[t]]
        bad_end=any(fresh(assessment(n),now) and not assessment(n).feasible for n in endpoints)
        measured_end=any(fresh(assessment(n),now) and not assessment(n).feasible and violation(n) for n in endpoints)
        bad_edge=fresh(a,now) and a.physical_traversability<=0
        measured_edge=bad_edge and violation(e)
        traversed=any(v>.5 for v in prop(e.traversability[ci].properties,'traversed')[:1])
        weight=e.traversability[ci].traversability_cost
        admitted=not measured_end and not measured_edge and (traversed or (not bad_end and not bad_edge)) and math.isfinite(weight) and weight>=0
        reasons=[]
        if measured_end:reasons.append('measured_endpoint')
        if measured_edge:reasons.append('measured_edge')
        if not traversed and bad_end:reasons.append('endpoint_infeasible')
        if not traversed and bad_edge:reasons.append('edge_infeasible')
        if not admitted:rejected.update(reasons or ['weight'])
        else:physical[f].append(t)
        if start in (f,t):
            incident.append(dict(from_idx=f,to_idx=t,from_xy=xy(nodes[f]),to_xy=xy(nodes[t]),
                                 admitted=admitted,reasons=reasons,assessment=plain(a) if a else None,
                                 properties={v.key:list(v.value) for v in e.properties}))
    toporeach=reach(topo,start);physreach=reach(physical,start)
    frontier=[];candidate=[]
    origin=[odom.pose.pose.position.x,odom.pose.pose.position.y] if odom else xy(nodes[start])
    for i,n in enumerate(scored.nodes):
        if ci>=len(n.trav_properties) or not n.trav_properties[ci].is_frontier:continue
        a=assessment(n);score=max(prop(n.properties,'frontier_scores'),default=.3)
        heading=max((h.predicted_utility for h in n.heading_candidates if h.traversable),default=None)
        blocked=fresh(a,row['ros_ns']) and (not a.feasible or a.physical_traversability<=0 or not math.isfinite(a.dynamic_risk) or a.dynamic_risk>=1.)
        eligible=not blocked and math.dist(origin,xy(n))>6. and math.isfinite(score) and score>1e-4 and (not n.heading_candidates or heading is not None)
        item=dict(index=i,position=xy(n),distance=math.dist(origin,xy(n)),score=score,utility=heading,
                  physical_blocked=blocked,eligible_before_backoff=eligible,topology_reachable=i in toporeach,physical_reachable=i in physreach,
                  attach_from_reachable=any(math.dist(xy(nodes[j]),xy(n))<6. for j in physreach))
        frontier.append(item)
        if eligible:candidate.append(item)
    reports.append(dict(wall_ns=row['wall_ns'],source_ns=now,ros_ns=row['ros_ns'],nodes=len(nodes),edges=len(g.edges),
                        current_idx=start,current_xy=xy(nodes[start]),odom_xy=origin,origin_gap=math.dist(origin,xy(nodes[start])),
                        current=plain(nodes[start]),topology_reachable=len(toporeach),physical_reachable=len(physreach),
                        rejected_edge_reasons=dict(rejected),incident=incident,frontiers=frontier,
                        candidates_before_backoff=len(candidate),reachable_candidates=sum(v['physical_reachable'] for v in candidate),
                        candidate_attach_possible=sum(v['attach_from_reachable'] for v in candidate),plan=row.get('plan'),
                        source_files={k:row[k] for k in ('raw','scored','odom') if k in row}))
out=dict(scope=__doc__,observer_errors=errors,snapshots=reports,
         limitation='Does not replay historical rejected edges, destination backoff, visited policy, virtual frontier costs or scoring-overlay receipt timing.')
args.output.write_text(json.dumps(out,indent=2,allow_nan=True)+'\n')
print(json.dumps(dict(output=str(args.output),snapshots=len(reports),observer_errors=len(errors))))
for i,r in enumerate(reports):
    if i%10==0 or i==len(reports)-1:
        print(json.dumps({k:r[k] for k in ('source_ns','nodes','edges','current_idx','current_xy','odom_xy','origin_gap','topology_reachable','physical_reachable','rejected_edge_reasons','candidates_before_backoff','reachable_candidates','candidate_attach_possible')}))
