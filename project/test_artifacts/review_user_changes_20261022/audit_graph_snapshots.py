"""Read exact saved graph pairs without contacting ROS or the simulator."""
from pathlib import Path
import collections
import hashlib
import json
import math
import sys
from rclpy.serialization import deserialize_message
from graphnav_msgs.msg import NavigationGraph
from nav_msgs.msg import Odometry
from rosidl_runtime_py.convert import message_to_ordereddict

root = Path(sys.argv[1] if len(sys.argv)>1 else '/home/ly/execution_continuity_review_20261022/v1')
trial = Path(sys.argv[2]) if len(sys.argv)>2 else root/'full/trial_20261022_155227'
pairdir = next((trial/'sessions').glob('session_*'))/'graph_pairs'
t0 = json.loads((trial/'fluency_audit/summary_420.json/closed_trial_summary.json').read_text())['query_start_bag_wall']
rows = [json.loads(s) for s in (pairdir/'snapshots.jsonl').read_text().splitlines()]
pairs = [r for r in rows if r.get('kind') == 'pair' and 'raw' in r]
print(json.dumps({'pair_contract_counts': dict(collections.Counter(json.dumps(r['contract'],sort_keys=True) for r in pairs))}),flush=True)
pairs = [r for r in pairs if all(r['contract'].values())]

def stamp(t): return t.sec+t.nanosec*1e-9
def prop(items, name, default=None):
    return next((list(x.value) for x in items if x.key == name), default)
def measured(items): return prop(items, 'physical_violation_observed') == [1.]
def assessment(items):
    return next((x for x in items if x.robot_profile_id == 'beamng_sbr'), None)
def fresh(a, now):
    return a is not None and a.observed_at.sec > 0 and math.isfinite(a.confidence) and .7 <= a.confidence <= 1 and 0 <= now-stamp(a.observed_at) <= 5.

out=[]
for elapsed in (10, 100, 170, 245, 260, 264, 270, 300, 350, 410):
    row = min(pairs, key=lambda r:abs(r['ros_ns']*1e-9-t0-elapsed))
    raw = deserialize_message((pairdir/row['raw']['file']).read_bytes(), NavigationGraph)
    scored = deserialize_message((pairdir/row['scored']['file']).read_bytes(), NavigationGraph)
    odom = deserialize_message((pairdir/row['odom']['file']).read_bytes(), Odometry)
    assert all(v for k,v in row['contract'].items()), row['contract']
    now = stamp(raw.header.stamp)
    idx = raw.current_node_idx
    classes = list(raw.trav_classes)
    # The runtime profile name is reported below; prefer its sole assessment.
    profile_counts = collections.Counter(a.robot_profile_id for n in scored.nodes for a in n.robot_traversability)
    profile = profile_counts.most_common(1)[0][0] if profile_counts else 'default'
    def assessment(items): return next((x for x in items if x.robot_profile_id == profile), None)
    ci=classes.index('default') if 'default' in classes else 0
    reasons=collections.Counter(); adjacency=collections.defaultdict(list); incident=[]
    for edge in raw.edges:
        if edge.from_idx >= len(scored.nodes) or edge.to_idx >= len(scored.nodes) or ci >= len(edge.traversability): continue
        a=assessment(edge.robot_traversability)
        traversed = bool(prop(edge.traversability[ci].properties,'traversed',[0.])[0] > .5)
        endpoint_ok=True; measured_endpoint=False
        for i in (edge.from_idx,edge.to_idx):
            n=scored.nodes[i]; na=assessment(n.robot_traversability)
            if fresh(na,now) and not na.feasible:
                endpoint_ok=False
                if measured(n.properties): measured_endpoint=True
        hard=fresh(a,now) and a.physical_traversability <= 0
        reason = ('measured_endpoint' if measured_endpoint else
                  'measured_edge' if hard and measured(edge.properties) else
                  'blocked' if not (traversed or (endpoint_ok and not hard)) else 'accepted')
        weight=edge.traversability[ci].traversability_cost
        if not math.isfinite(weight) or weight < 0: reason='invalid_weight'
        reasons[reason]+=1
        if reason=='accepted':
            adjacency[edge.from_idx].append(edge.to_idx); adjacency[edge.to_idx].append(edge.from_idx)
        if idx in (edge.from_idx,edge.to_idx):
            incident.append(dict(from_idx=edge.from_idx,to_idx=edge.to_idx,reason=reason,traversed=traversed,
                                 edge=message_to_ordereddict(edge)))
    reached={idx};queue=[idx]
    for i in queue:
        for j in adjacency[i]:
            if j not in reached:reached.add(j);queue.append(j)
    frontiers=[]
    for i,n in enumerate(scored.nodes):
        if ci >= len(n.trav_properties) or not n.trav_properties[ci].is_frontier: continue
        a=assessment(n.robot_traversability)
        score=max(prop(n.properties,'frontier_scores',[.3]))
        distance=math.hypot(n.pose.position.x-odom.pose.pose.position.x,n.pose.position.y-odom.pose.pose.position.y)
        physical_ok=not (fresh(a,now) and (not a.feasible or a.physical_traversability<=0 or a.dynamic_risk>=1))
        eligible=physical_ok and distance>6 and math.isfinite(score) and score>1e-4 and (not n.heading_candidates or any(h.traversable for h in n.heading_candidates))
        frontiers.append(dict(index=i,reached=i in reached,eligible_before_backoff=eligible,distance=distance,score=score,
                              pose=message_to_ordereddict(n.pose),physical=message_to_ordereddict(a) if a else None))
    result=dict(elapsed=row['ros_ns']*1e-9-t0,key=row['key'],files={k:row[k] for k in ('raw','scored','odom')},
                contracts=row['contract'],profile=profile,classes=classes,nodes=len(scored.nodes),edges=len(raw.edges),
                current_idx=idx,current=message_to_ordereddict(scored.nodes[idx]),odom=message_to_ordereddict(odom),
                plan=row.get('plan'),filter_counts=dict(reasons),reachable_nodes=len(reached),
                reachable_frontiers=sum(f['reached'] for f in frontiers),
                eligible_reachable_frontiers=sum(f['reached'] and f['eligible_before_backoff'] for f in frontiers),
                frontiers=frontiers,incident=incident)
    out.append(result)
    print(json.dumps({k:result[k] for k in ('elapsed','profile','nodes','edges','current_idx','filter_counts','reachable_nodes','reachable_frontiers','eligible_reachable_frontiers')}), flush=True)
(trial/'fluency_audit/graph_topology_audit.json').write_text(json.dumps(out,indent=2))
