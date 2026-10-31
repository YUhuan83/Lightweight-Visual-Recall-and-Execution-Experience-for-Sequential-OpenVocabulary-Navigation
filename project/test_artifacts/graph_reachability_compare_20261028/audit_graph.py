"""Compare finite raw edges to update_graph physical gates on closed v57 snapshots.

This emulates the source's edge admission; it does not reproduce transient
rejected_edges_ or the planner's candidate scoring/smoothing state.
"""
import collections
import json
import math
from pathlib import Path
from rclpy.serialization import deserialize_message
from graphnav_msgs.msg import NavigationGraph

base = Path('/home/ly/execution_continuity_review_20261022/graph_reachability_compare_20261028')
goal = (748.8052371300828,-130.0831851365844)

def props(items):
    return {p.key: list(p.value) for p in items}

def profile(items):
    return next((x for x in items if x.robot_profile_id == 'default'), None)

def fresh(a, stamp):
    if a is None or a.observed_at.sec <= 0 or a.observed_at.nanosec >= 1000000000:
        return False
    age = (stamp - (a.observed_at.sec * 1000000000 + a.observed_at.nanosec)) / 1e9
    return math.isfinite(a.confidence) and .7 <= a.confidence <= 1 and 0 <= age <= 5

def reachable(current, adjacency):
    seen = {current}
    q = collections.deque([current])
    while q:
        for n in adjacency[q.popleft()]:
            if n not in seen:
                seen.add(n)
                q.append(n)
    return seen

out = []
for line in (base / 'inputs.txt').read_text().splitlines():
    raw, scored, gx, gy, gz, stamp = line.split()
    g = deserialize_message(Path(raw).read_bytes(), NavigationGraph)
    s = deserialize_message(Path(scored).read_bytes(), NavigationGraph)
    assert g.header == s.header and len(g.nodes) == len(s.nodes)
    g.nodes = s.nodes
    idx = g.trav_classes.index('default')
    current = g.current_node_idx
    header_ns = g.header.stamp.sec * 1000000000 + g.header.stamp.nanosec
    raw_adj = collections.defaultdict(list)
    filtered_adj = collections.defaultdict(list)
    rejected = collections.Counter()
    blocked_edges = []
    finite_edges = {}
    for e in g.edges:
        if e.from_idx >= len(g.nodes) or e.to_idx >= len(g.nodes) or idx >= len(e.traversability):
            continue
        cost = e.traversability[idx].traversability_cost
        if not math.isfinite(cost) or cost < 0:
            rejected['nonfinite_cost'] += 1
            continue
        raw_adj[e.from_idx].append(e.to_idx)
        finite_edges[(e.from_idx, e.to_idx)] = e
        traversed = props(e.traversability[idx].properties).get('traversed', [0])[0] > .5
        ea = profile(e.robot_traversability)
        hard = fresh(ea, header_ns) and ea.physical_traversability <= 0
        endpoint_bad = False
        measured_endpoint = False
        for i in (e.from_idx, e.to_idx):
            n = g.nodes[i]
            a = profile(n.robot_traversability)
            if fresh(a, header_ns) and not a.feasible:
                endpoint_bad = True
                if props(n.properties).get('physical_violation_observed') == [1.0]:
                    measured_endpoint = True
        measured_edge = hard and props(e.properties).get('physical_violation_observed') == [1.0]
        if not measured_endpoint and not measured_edge and (traversed or (not endpoint_bad and not hard)):
            filtered_adj[e.from_idx].append(e.to_idx)
        else:
            reasons = [name for name, val in [('measured_endpoint', measured_endpoint), ('measured_edge', measured_edge), ('endpoint_infeasible', endpoint_bad and not traversed), ('edge_infeasible', hard and not traversed)] if val]
            for name in reasons: rejected[name] += 1
            blocked_edges.append({'from': e.from_idx, 'to': e.to_idx, 'reasons': reasons, 'physical': props(e.properties)})
    raw_seen, filt_seen = reachable(current, raw_adj), reachable(current, filtered_adj)
    raw_undirected=collections.defaultdict(list); filtered_undirected=collections.defaultdict(list)
    for adjacency, symmetric in ((raw_adj,raw_undirected),(filtered_adj,filtered_undirected)):
        for a,bs in adjacency.items():
            for b in bs:symmetric[a].append(b);symmetric[b].append(a)
    raw_us=reachable(current,raw_undirected);filter_us=reachable(current,filtered_undirected)
    cp = g.nodes[current].pose.position
    current_distance = math.hypot(cp.x - goal[0], cp.y - goal[1])
    def describe(i):
        n = g.nodes[i]; p = n.pose.position
        t = n.trav_properties[idx] if idx < len(n.trav_properties) else None
        return {'id': i, 'xy': [p.x, p.y], 'goal_distance': math.hypot(p.x-goal[0], p.y-goal[1]), 'frontier': bool(t and t.is_frontier), 'finite_raw_reachable': i in raw_seen, 'physical_filter_reachable': i in filt_seen,'raw_undirected':i in raw_us,'filtered_undirected':i in filter_us}
    nodes = [describe(i) for i in range(len(g.nodes))]
    forward = sorted((n for n in nodes if n['goal_distance'] < current_distance - 5), key=lambda n:n['goal_distance'])
    cuts = [e for e in blocked_edges if e['from'] in filt_seen and e['to'] not in filt_seen]
    def raw_path(target):
        q=collections.deque([current]); parent={current:None}
        while q and target not in parent:
            i=q.popleft()
            for j in raw_adj[i]:
                if j not in parent:
                    parent[j]=i; q.append(j)
        if target not in parent: return None
        ids=[]; p=target
        while parent[p] is not None:
            ids.append((parent[p],p)); p=parent[p]
        return [{'from': a, 'to': b, 'blocked': next((e for e in blocked_edges if e['from']==a and e['to']==b), None)} for a,b in reversed(ids)]
    sample = [n for n in forward if n['frontier'] and n['finite_raw_reachable'] and not n['physical_filter_reachable']][:2]
    out.append({'pair': Path(scored).name, 'current': describe(current), 'nodes': len(nodes), 'edges': len(g.edges), 'finite_raw_reachable': len(raw_seen), 'physical_filter_reachable': len(filt_seen), 'forward_nodes_5m': len(forward), 'forward_raw_reachable': sum(n['finite_raw_reachable'] for n in forward), 'forward_filter_reachable': sum(n['physical_filter_reachable'] for n in forward), 'forward_frontiers_raw_reachable': sum(n['frontier'] and n['finite_raw_reachable'] for n in forward), 'forward_frontiers_filter_reachable': sum(n['frontier'] and n['physical_filter_reachable'] for n in forward), 'nearest_goal_nodes': forward[:12], 'sample_paths': [{'target':n,'path':raw_path(n['id'])} for n in sample], 'cut_edges':cuts, 'rejected_counts_overlapping': dict(rejected), 'blocked_edges': blocked_edges})
    out[-1]['forward_frontiers_raw_undirected']=sum(n['frontier'] and n['raw_undirected'] for n in forward)
    out[-1]['forward_frontiers_filtered_undirected']=sum(n['frontier'] and n['filtered_undirected'] for n in forward)
(base / 'graph_connectivity.json').write_text(json.dumps(out, indent=2, allow_nan=False))
for x in out:
    print(json.dumps({k:v for k,v in x.items() if k not in ('blocked_edges','cut_edges','nearest_goal_nodes','sample_paths')}, ensure_ascii=False))
