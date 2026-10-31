"""Read exact raw/scored pair 156; characterize the missing forward connection."""
import collections,json,math
from pathlib import Path
from rclpy.serialization import deserialize_message
from graphnav_msgs.msg import NavigationGraph
root=Path('/home/ly/execution_continuity_review_20261022/frontier_endpoint_contract_review')
line=next(s for s in (root/'recorded_graphs.txt').read_text().splitlines() if 'pair_000156_' in s)
raw,scored,*_=line.split()
g=deserialize_message(Path(raw).read_bytes(),NavigationGraph)
s=deserialize_message(Path(scored).read_bytes(),NavigationGraph)
assert g.header==s.header and len(g.nodes)==len(s.nodes)
g.nodes=s.nodes
idx=g.trav_classes.index('default');current=g.current_node_idx
def path_to(target,finite=False,undirected=False):
    adj=collections.defaultdict(list)
    for i,e in enumerate(g.edges):
        if e.from_idx>=len(g.nodes) or e.to_idx>=len(g.nodes) or idx>=len(e.traversability):continue
        cost=e.traversability[idx].traversability_cost
        if finite and not(math.isfinite(cost) and cost>=0):continue
        adj[e.from_idx].append((e.to_idx,i,False))
        if undirected:adj[e.to_idx].append((e.from_idx,i,True))
    q=collections.deque([current]);seen={current:None}
    while q:
        node=q.popleft()
        for other,i,reversed_edge in adj[node]:
            if other not in seen:seen[other]=(node,i,reversed_edge);q.append(other)
    if target not in seen:return None,len(seen)
    out=[];node=target
    while seen[node] is not None:
        parent,i,rev=seen[node];e=g.edges[i]
        out.append(dict(from_idx=parent,to_idx=node,used_reverse=rev,stored_from=e.from_idx,stored_to=e.to_idx,cost=str(e.traversability[idx].traversability_cost),properties={p.key:list(p.value) for p in e.traversability[idx].properties},physical_metadata={p.key:list(p.value) for p in e.properties},node_metadata={str(n):{p.key:list(p.value) for p in g.nodes[n].properties} for n in (parent,node)},physical=[dict(profile=a.robot_profile_id,trav=a.physical_traversability,confidence=a.confidence,stamp=[a.observed_at.sec,a.observed_at.nanosec]) for a in e.robot_traversability]))
        node=parent
    return list(reversed(out)),len(seen)
out=dict(current=current,current_position=[g.nodes[current].pose.position.x,g.nodes[current].pose.position.y],nodes=len(g.nodes),edges=len(g.edges),cases=[])
for target in (19,27,53,314):
    n=g.nodes[target];r=dict(target=target,position=[n.pose.position.x,n.pose.position.y],properties={p.key:list(p.value) for p in n.properties})
    for name,finite,undirected in [('raw_directed',False,False),('finite_directed',True,False),('finite_undirected',True,True)]:
        path,count=path_to(target,finite,undirected);r[name]=dict(reachable=path is not None,reachable_count=count,path=path)
    out['cases'].append(r)
(root/'forward_connectivity.json').write_text(json.dumps(out,indent=2,allow_nan=False))
print(json.dumps(out,indent=2,allow_nan=False))
