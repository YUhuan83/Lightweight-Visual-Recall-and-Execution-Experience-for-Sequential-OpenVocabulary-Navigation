"""Bounded target validity and actual recovery audit on the now closed field bag."""
from pathlib import Path
import bisect,collections,json,math,sqlite3,uuid,struct,hashlib
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

root=Path('/home/ly/visible_pursuit_repair_20261017');field=root/'field'
r=json.loads((field/'closed_review.json').read_text());s=Path(r['session']);g=s/'goal_001'
driver=sorted(root.glob('field_yaw_left90_search_*'))[-1]
assert (driver/'result.json').exists() and (s/'session_result.json').exists() and (g/'result.json').exists()
events=[json.loads(x) for x in (g/'events.jsonl').read_text().splitlines() if x]
plans=[x for x in events if x.get('kind')=='plan_status']
native=[json.loads(x) for x in (g/'native_samples.jsonl').read_text().splitlines() if x]
db=sqlite3.connect(next((g/'interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
def records(connection,topic):
    pair=connection.execute('select id,type from topics where name=?',(topic,)).fetchone()
    if not pair:return []
    cls=get_message(pair[1])
    return [(t,deserialize_message(raw,cls)) for t,raw in connection.execute('select timestamp,data from messages where topic_id=? order by timestamp',(pair[0],))]
routes=[dict(received_ns=t,**json.loads(m.data)) for t,m in records(db,'/spot1/graphnav_planner/route_status')]
grouped=collections.defaultdict(list)
for row in routes:grouped[row.get('exploration_id')].append(row)
target_stats=[]
for identity,rows in grouped.items():
    if not identity:continue
    progress=r['identity_progress'][identity]
    target_stats.append(dict(identity=identity,goal=rows[0]['goal'],duration_s=(rows[-1]['received_ns']-rows[0]['received_ns'])/1e9,
        count=len(rows),complete=sum(bool(x.get('complete')) for x in rows),**{k:v for k,v in progress.items() if k!='goal'}))
longest=max(target_stats,key=lambda x:x['duration_s'])
samples=[rows for key,rows in grouped.items() if key==longest['identity']][0]
sensor=sqlite3.connect(next((s/'sensor_interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
tid,typ=sensor.execute("select id,type from topics where name='/spot1/scored_nav_graph'").fetchone();cls=get_message(typ)
topology_id,topology_typ=sensor.execute("select id,type from topics where name='/spot1/nav_graph'").fetchone()
topology_index={}
for mid,header in sensor.execute('select id,substr(data,1,12) from messages where topic_id=?',(topology_id,)):
    sec,nano=struct.unpack_from('<iI' if header[1]==1 else '>iI',header,4)
    topology_index.setdefault(sec*10**9+nano,[]).append(mid)
evidence=field/'target_snapshots';evidence.mkdir(exist_ok=True)
target_uuid=longest['identity'].split(':')[0];snapshots=[]
for at in [samples[0]['received_ns'],samples[len(samples)//2]['received_ns'],samples[-1]['received_ns']]:
    row=sensor.execute('select timestamp,data from messages where topic_id=? and timestamp<=? order by timestamp desc limit 1',(tid,at)).fetchone()
    if row is None:continue
    ts,raw=row;graph=deserialize_message(raw,cls)
    score_file=evidence/(str(ts)+'_scored.cdr');score_file.write_bytes(raw);del raw
    graph_ns=graph.header.stamp.sec*10**9+graph.header.stamp.nanosec
    topology_rows=topology_index.get(graph_ns,[])
    assert topology_rows,'Missing exact topology acquisition'
    topology=None
    for mid in reversed(topology_rows):
        raw=sensor.execute('select data from messages where id=?',(mid,)).fetchone()[0]
        trial_graph=deserialize_message(raw,get_message(topology_typ))
        if graph.header==trial_graph.header and len(graph.nodes)==len(trial_graph.nodes) and all(a.uuid==b.uuid and a.pose==b.pose for a,b in zip(graph.nodes,trial_graph.nodes)):
            topology=trial_graph;break
    assert topology is not None,(ts,graph_ns,len(graph.nodes),len(topology_rows))
    topology_file=evidence/(str(ts)+'_topology.cdr');topology_file.write_bytes(raw);del raw
    assert graph.header==topology.header and len(graph.nodes)==len(topology.nodes),(ts,graph_ns,str(graph.header),str(topology.header),len(graph.nodes),len(topology.nodes))
    assert all(a.uuid==b.uuid and a.pose==b.pose for a,b in zip(graph.nodes,topology.nodes))
    ci=graph.trav_classes.index('default')
    candidates=sorted(graph.nodes,key=lambda n:math.dist([n.pose.position.x,n.pose.position.y],longest['goal']))[:3]
    exact=[n for n in graph.nodes if str(uuid.UUID(bytes=bytes(n.uuid.id)))==target_uuid]
    def describe(n):
        a=next((a for a in n.robot_traversability if a.robot_profile_id=='default'),None)
        return dict(uuid=str(uuid.UUID(bytes=bytes(n.uuid.id))),xy=[n.pose.position.x,n.pose.position.y],
            frontier=n.trav_properties[ci].is_frontier,properties={p.key:list(p.value) for p in n.trav_properties[ci].properties},
            assessment=None if a is None else dict(feasible=a.feasible,confidence=a.confidence,physical=a.physical_traversability,dynamic=a.dynamic_risk,
                source_age_s=(ts-a.observed_at.sec*10**9-a.observed_at.nanosec)/1e9),
            best_score=max((max(p.value) for p in n.properties if p.key=='frontier_scores' and p.value),default=None))
    topo_exact=[n for n in topology.nodes if str(uuid.UUID(bytes=bytes(n.uuid.id)))==target_uuid]
    snapshots.append(dict(received_ns=ts,graph_source_ns=graph_ns,goal_uuid_present=bool(exact),exact=list(map(describe,exact)),nearest=list(map(describe,candidates)),
        matching_raw_topology=True,raw_exact=list(map(describe,topo_exact)),cdr_evidence=[dict(path=str(f),sha256=hashlib.sha256(f.read_bytes()).hexdigest()) for f in (score_file,topology_file)]))
    del graph,topology
recoveries=[]
for p in plans:
    if p.get('status')!='recovery_started' or p.get('measured_corridor') is not True:continue
    start=p['stamp_ns']/1e9
    terminal=next((x for x in plans if x['stamp_ns']>p['stamp_ns'] and (x.get('status') in ('recovery_done','recovery_failed') or (x.get('status')=='recovery_started' and x.get('measured_corridor') is True))),None)
    end=terminal['stamp_ns']/1e9 if terminal else start+15
    samples=[x for x in native if start<=x['wall_time']<=end]
    velocity=[]
    for x in samples:
        if isinstance(x.get('direction'),list) and isinstance(x.get('velocity'),list):
            d=x['direction'];v=x['velocity'];norm=math.hypot(*d[:2])
            velocity.append((d[0]*v[0]+d[1]*v[1])/norm if norm else 0.)
    relevant=[x for x in plans if start<=x['stamp_ns']/1e9<=end]
    recoveries.append(dict(start=p,terminal=terminal,samples=len(samples),distance_m=sum(math.dist(a['position'][:2],b['position'][:2]) for a,b in zip(samples,samples[1:])),
        signed_speed_min=min(velocity,default=None),reverse_samples=sum(v<-.15 for v in velocity),status_changes=relevant))
output=dict(targets=target_stats,longest_target=longest,longest_target_snapshots=snapshots,recoveries=recoveries,
    scope='Public graph, route and native evidence; no assertion of private action costmap identity')
(field/'target_progress_review.json').write_text(json.dumps(output,indent=2)+'\n')
print(json.dumps(dict(longest_target=longest,snapshots=snapshots,recoveries=[{k:v for k,v in x.items() if k!='status_changes'} for x in recoveries]),indent=2))
