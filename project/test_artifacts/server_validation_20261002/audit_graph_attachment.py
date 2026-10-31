"""Inspect remembered-goal graph attachment and the exact obstacle rejection."""
from pathlib import Path
import sys,json,subprocess,sqlite3,math,zipfile
sys.path.insert(0,'/home/ly')
from session_tools_20261002 import ROOT,WS,runtime_env
if '--runtime-child' not in sys.argv:
    raise SystemExit(subprocess.run([str(WS/'.venv/bin/python'),__file__,'--runtime-child'],env=runtime_env()).returncode)
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from wildos_nav2_adapter.graph_route import footprint_blockers,cell_cost
db=sqlite3.connect('file:'+str(ROOT/'body_collision_full_trial1/rosbag/rosbag_0.db3')+'?mode=ro',uri=True)
topics={name:(tid,typ) for tid,name,typ in db.execute('select id,name,type from topics')}
out=ROOT/'mechanism_audit'
audit=json.loads((out/'mechanism.json').read_text())
p=audit['bag']['/wildos/full_planned_path'][0]
target=json.loads((ROOT/'body_collision_full_trial1/full_test.json').read_text())['station_views_before_departure'][0]['position']
def read(topic,ts):
    tid,typ=topics[topic]
    t,b=db.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',(tid,ts)).fetchone()
    return deserialize_message(b,get_message(typ))
report={'target':target}
for topic in ('/spot1/nav_graph','/spot1/scored_nav_graph'):
    m=read(topic,p['timestamp'])
    data=[]
    for i,n in enumerate(m.nodes):
        pos=[n.pose.position.x,n.pose.position.y,n.pose.position.z]
        data.append(dict(index=i,position=pos,xy=math.dist(pos[:2],target[:2]),xyz=math.dist(pos,target),
            frontier=[t.is_frontier for t in n.trav_properties],
            profiles=[dict(profile=t.robot_profile_id,feasible=t.feasible,physical=t.physical_traversability,confidence=t.confidence) for t in n.robot_traversability]))
    closest=sorted(data,key=lambda n:n['xy'])[:8]
    ids={d['index'] for d in closest}
    edges=[dict(a=e.from_idx,b=e.to_idx,cost=[t.traversability_cost for t in e.traversability]) for e in m.edges if e.from_idx in ids or e.to_idx in ids]
    import heapq
    feasible={i for i,n in enumerate(m.nodes) if not n.robot_traversability or any(t.robot_profile_id=='default' and t.feasible for t in n.robot_traversability)}
    adjacency={}
    for e in m.edges:
        assessment=next((t for t in e.robot_traversability if t.robot_profile_id=='default'),None)
        if e.from_idx not in feasible or e.to_idx not in feasible or not e.traversability or (assessment and assessment.physical_traversability<=0):continue
        weight=float(e.traversability[0].traversability_cost)
        if assessment:weight+=12*assessment.dynamic_risk+4*(1-assessment.confidence)
        if not math.isfinite(weight) or weight<0:continue
        adjacency.setdefault(e.from_idx,[]).append((e.to_idx,weight))
        adjacency.setdefault(e.to_idx,[]).append((e.from_idx,weight))
    best={m.current_node_idx:0.};parents={};queue=[(0.,m.current_node_idx)]
    while queue:
        cost,u=heapq.heappop(queue)
        if cost!=best.get(u):continue
        for v,w in adjacency.get(u,[]):
            nc=cost+w
            if nc<best.get(v,math.inf):best[v]=nc;parents[v]=u;heapq.heappush(queue,(nc,v))
    anchors=[v for v in data if v['xyz']<6 and v['index'] in feasible and v['index'] in best]
    known=None
    if anchors:
        anchor=min(anchors,key=lambda v:best[v['index']]+v['xyz'])
        ids_path=[anchor['index']]
        while ids_path[-1]!=m.current_node_idx:ids_path.append(parents[ids_path[-1]])
        ids_path.reverse()
        known=dict(anchor=anchor['index'],cost=best[anchor['index']]+anchor['xyz'],
            indices=ids_path,points=[data[i]['position'] for i in ids_path]+[target])
    # Compare structural connectivity with finite costs and platform filtering.
    variants={}
    edge_values=[]
    for e in m.edges:
        a=next((t for t in e.robot_traversability if t.robot_profile_id=='default'),None)
        if not e.traversability:continue
        w=float(e.traversability[0].traversability_cost)
        edge_values.append(dict(a=e.from_idx,b=e.to_idx,cost=w,physical=a.physical_traversability if a else None,
            clearance=a.minimum_clearance if a else None,slope=a.maximum_slope if a else None,
            step=a.maximum_step_height if a else None,roughness=a.roughness if a else None))
    for mode in ('finite_raw','node_only','edge_only','filtered'):
        adj={}
        for e in edge_values:
            if not math.isfinite(e['cost']) or e['cost']<0:continue
            if mode in ('node_only','filtered') and (e['a'] not in feasible or e['b'] not in feasible):continue
            if mode in ('edge_only','filtered') and e['physical'] is not None and e['physical']<=0:continue
            adj.setdefault(e['a'],[]).append((e['b'],e['cost']))
            adj.setdefault(e['b'],[]).append((e['a'],e['cost']))
        costs={m.current_node_idx:0.};prev={};q=[(0.,m.current_node_idx)]
        while q:
            cost,u=heapq.heappop(q)
            if cost!=costs.get(u):continue
            for v,w in adj.get(u,[]):
                if cost+w<costs.get(v,math.inf):costs[v]=cost+w;prev[v]=u;heapq.heappush(q,(cost+w,v))
        options=[v for v in data if v['xyz']<6 and v['index'] in costs and (mode not in ('node_only','filtered') or v['index'] in feasible)]
        result=dict(reachable_nodes=len(costs),known_target_reachable=bool(options))
        if options:
            anchor=min(options,key=lambda v:costs[v['index']]+v['xyz'])
            ids_path=[anchor['index']]
            while ids_path[-1]!=m.current_node_idx:ids_path.append(prev[ids_path[-1]])
            ids_path.reverse()
            result.update(cost=costs[anchor['index']]+anchor['xyz'],indices=ids_path,
                points=[data[i]['position'] for i in ids_path]+[target],
                rejected_edges=[e for a,b in zip(ids_path,ids_path[1:]) for e in edge_values if e['a']==a and e['b']==b and e['physical']==0],
                rejected_nodes=[data[i] for i in ids_path if i not in feasible])
        variants[mode]=result
    all_adjacency={}
    for e in m.edges:
        all_adjacency.setdefault(e.from_idx,[]).append(e.to_idx)
        all_adjacency.setdefault(e.to_idx,[]).append(e.from_idx)
    def component(start,adj):
        seen={start};todo=[start]
        while todo:
            for item in adj.get(todo.pop(),[]):
                v=item[0] if isinstance(item,tuple) else item
                if v not in seen:seen.add(v);todo.append(v)
        return seen
    source_component=component(m.current_node_idx,all_adjacency)
    target_component=component(closest[0]['index'],all_adjacency)
    filtered_component=component(m.current_node_idx,adjacency)
    near=sorted((math.dist(data[a]['position'][:2],data[b]['position'][:2]),a,b) for a in source_component for b in target_component)[:4]
    connectivity=dict(edge_count=len(m.edges),physical_zero_edges=sum(e['physical']==0 for e in edge_values),
        nonfinite_edges=sum(not math.isfinite(e['cost']) for e in edge_values),
        infeasible_nodes=len(m.nodes)-len(feasible),variants=variants,raw_component=len(source_component),
        filtered_component=len(filtered_component),target_component=len(target_component),
        raw_target_connected=closest[0]['index'] in source_component,
        nearest_components=[dict(gap=d,a=a,b=b,from_position=data[a]['position'],to_position=data[b]['position']) for d,a,b in near],
        filtered_source_nodes=[data[i] for i in filtered_component] if len(filtered_component)<20 else [])
    report[topic]=dict(current_node_idx=m.current_node_idx,node_count=len(m.nodes),closest=closest,edges=edges,known_route=known,connectivity=connectivity,
        current=data[m.current_node_idx] if 0<=m.current_node_idx<len(data) else None,
        z_range=[min(n['position'][2] for n in data),max(n['position'][2] for n in data)])
# Map changes close to the formerly valid path
tid,typ=topics['/global_costmap/costmap'];cls=get_message(typ);previous=None;changes=[]
for ts,b in db.execute('select timestamp,data from messages where topic_id=? and timestamp>=? and timestamp<? order by timestamp',(tid,p['timestamp']-1e9,p['timestamp']+9e9)):
    m=deserialize_message(b,cls)
    collisions=[dict(index=i,point=pt,blockers=footprint_blockers(m,pt,h,limit=1),center_cost=cell_cost(m,*pt)) for i,(pt,h) in enumerate(zip(p['points'],p['headings'])) if footprint_blockers(m,pt,h,limit=1)]
    if collisions!=previous:
        changes.append(dict(after_path=(ts-p['timestamp'])/1e9,collisions=collisions));previous=collisions
report['prefix_grid_changes']=changes
(out/'graph_attachment.json').write_text(json.dumps(report,indent=2))
with zipfile.ZipFile(out/'graph_sources.zip','w',zipfile.ZIP_DEFLATED) as z:
    for package in ('graphnav_planner','graphnav_mapper','graphnav_msgs'):
        for file in (WS/'src/nebula2-wildos'/package).rglob('*'):
            if file.is_file() and file.suffix in ('.cpp','.hpp','.yaml','.msg','.srv'):
                z.write(file,package+'/'+str(file.relative_to(WS/'src/nebula2-wildos'/package)))
print(json.dumps({k:({key:val for key,val in v.items() if key!='edges'} if isinstance(v,dict) else v) for k,v in report.items()},indent=2),flush=True)
db.close()

