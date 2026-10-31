"""Closed bag audit. Does not launch DDS, vehicles or change production files."""
from pathlib import Path
from bisect import bisect_right
from collections import Counter
import copy, hashlib, json, math, sqlite3, struct, subprocess, uuid
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message

ROOT = Path('/home/ly/exploration_regression_audit_20261016')
SESSION = Path('/home/ly/wildos-console/audit_results/session_20261016_145143')
GOAL = SESSION/'goal_001'
assert Path('/home/ly/wildos-console/field_frontier_cost_search_20261016_145142/result.json').exists()
assert (SESSION/'session_result.json').exists() and (GOAL/'result.json').exists()
ROOT.mkdir(exist_ok=True)
(ROOT/'cdr').mkdir(exist_ok=True)

def ns(m): return m.header.stamp.sec*10**9+m.header.stamp.nanosec
def xy(p): return [p.x,p.y]
def uid(n): return str(uuid.UUID(bytes=bytes(n.uuid.id)))
def read_bag(folder, names):
    result={name:[] for name in names}
    with sqlite3.connect(next(folder.glob('*.db3')).as_uri()+'?mode=ro',uri=True) as db:
        for tid,name,typ in db.execute('select id,name,type from topics'):
            if name not in result: continue
            kind=get_message(typ)
            result[name]=[(ts,deserialize_message(raw,kind)) for ts,raw in db.execute(
                'select timestamp,data from messages where topic_id=? order by timestamp',(tid,))]
    return result

data=read_bag(GOAL/'interfaces',[
    '/spot1/graphnav_planner/route_status','/spot1/graphnav_planner/path',
    '/spot1/graphnav_planner/rejected_route_endpoint','/spot1/graphnav_planner/retired_exploration',
    '/spot1/odom','/spot1/object_search/query'])
decisions={json.loads(m.data)['stamp_ns']:json.loads(m.data)
           for _,m in data['/spot1/graphnav_planner/route_status']}
paths={ns(m):m for _,m in data['/spot1/graphnav_planner/path']}
retired=[(ts,json.loads(m.data)) for ts,m in data['/spot1/graphnav_planner/retired_exploration']]
query_received=data['/spot1/object_search/query'][0][0]
odom=data['/spot1/odom']; odom_times=[ts for ts,_ in odom]
def latest_odom(cutoff):
    i=bisect_right(odom_times,cutoff)-1
    return odom[i] if i>=0 else (None,None)

rejections=[]; commands=[]
for ts,message in data['/spot1/graphnav_planner/rejected_route_endpoint']:
    source=ns(message); decision=decisions.get(source); path=paths.get(source)
    accepted=next(((t,r) for t,r in retired if r['reason']=='vehicle_route_rejected'
        and decision and r['exploration_id']==decision.get('exploration_id') and abs(t-ts)<10**9),None)
    assert decision and path and accepted, ('Unmatched rejection',ts,source)
    now=accepted[1]['stamp_ns']; odom_ts,o=latest_odom(now)
    assert o and o.header.frame_id==message.header.frame_id and 0<=now-ns(o)<=5*10**9
    destination=decision['goal']; endpoint=xy(message.pose.position); failure=xy(o.pose.pose.position)
    pts=[xy(p.pose.position) for p in path.poses]
    row=dict(received_ns=ts,now_ns=now,source_stamp_ns=source,identity=decision['exploration_id'],
        goal=destination,endpoint=endpoint,failure_position=failure,path=pts,
        odom_receipt_gap_s=(now-odom_ts)/1e9,odom_source_age_s=(now-ns(o))/1e9)
    rejections.append(row)
    values=[now,source,*endpoint,*destination,*failure,len(pts),*(v for p in pts for v in p)]
    commands.append((now,0,'R '+' '.join(map(str,values))))

sensor=sqlite3.connect(next((SESSION/'sensor_interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True)
topic_ids={name:(tid,get_message(typ)) for tid,name,typ in sensor.execute('select id,name,type from topics')
           if name in ('/spot1/nav_graph','/spot1/scored_nav_graph')}
topo_id,topo_type=topic_ids['/spot1/nav_graph']; score_id,score_type=topic_ids['/spot1/scored_nav_graph']
# Header is the first message field: index CDR stamp bytes without allocating all graphs.
topology={}
for mid,header in sensor.execute('select id,substr(data,1,12) from messages where topic_id=?',(topo_id,)):
    sec,nano=struct.unpack_from('<iI' if header[1]==1 else '>iI',header,4)
    topology[sec*10**9+nano]=mid
snapshots=[]; missing=0; aggregate=Counter(); last_sample=0
for ts,blob in sensor.execute('select timestamp,data from messages where topic_id=? order by timestamp',(score_id,)):
    if ts<query_received: continue
    near_rejection=any(0<=ts-r['now_ns']<1_500_000_000 for r in rejections)
    if ts-last_sample<10**10 and not near_rejection: continue
    if snapshots and ts-snapshots[-1]['received_ns']<800_000_000: continue
    score=deserialize_message(blob,score_type)
    mid=topology.get(ns(score))
    graph=deserialize_message(sensor.execute('select data from messages where id=?',(mid,)).fetchone()[0],topo_type) if mid is not None else None
    if (graph is None or graph.current_node_idx!=score.current_node_idx or
        len(graph.nodes)!=len(score.nodes) or any(list(a.uuid.id)!=list(b.uuid.id) or a.pose!=b.pose
        for a,b in zip(graph.nodes,score.nodes))):
        missing+=1; continue
    assert ns(graph)==ns(score), 'CDR header index mismatch'
    combined=score; combined.edges=graph.edges
    if 'default' not in combined.trav_classes: continue
    ci=combined.trav_classes.index('default'); _,o=latest_odom(ts)
    origin=xy(o.pose.pose.position) if o and 0<=ts-ns(o)<=500000000 else xy(combined.nodes[combined.current_node_idx].pose.position)
    counts=Counter(); nodes=[]; eligible=[]
    for index,node in enumerate(combined.nodes):
        if len(node.trav_properties)<=ci or not node.trav_properties[ci].is_frontier: continue
        counts['frontiers']+=1
        a=next((v for v in node.robot_traversability if v.robot_profile_id=='default'),None)
        conf=a.confidence if a else 0.
        astamp=a.observed_at.sec*10**9+a.observed_at.nanosec if a else 0
        age=(ts-astamp)/1e9
        fresh=a and astamp>0 and 0<=age<=5. and math.isfinite(conf) and 0<=conf<=1
        veto=bool(fresh and conf>=.7 and (not a.feasible or not math.isfinite(a.physical_traversability)
             or a.physical_traversability<=0 or not math.isfinite(a.dynamic_risk) or a.dynamic_risk>=1))
        scoremax=next((max(p.value) for p in node.properties if p.key=='frontier_scores' and p.value),.3)
        headings=[h for h in node.heading_candidates if h.traversable]
        hard_heading=bool(node.heading_candidates and not headings)
        distance=math.dist(origin,xy(node.pose.position))
        legacy_ok=not veto and distance>3. and math.isfinite(scoremax) and scoremax>1e-4
        current_ok=legacy_ok and not hard_heading
        counts['eligible_before_heading_gate']+=legacy_ok
        counts['eligible_before_backoff']+=current_ok
        counts['low_confidence_heading_veto']+=bool(legacy_ok and hard_heading and conf<.7)
        counts['stale_heading_veto']+=bool(legacy_ok and hard_heading and not fresh)
        counts['fresh_physical_veto']+=veto
        if current_ok: eligible.append(index)
        nodes.append(dict(index=index,uuid=uid(node),position=xy(node.pose.position),distance=distance,
             score=scoremax,confidence=conf,physical_feasible=bool(a.feasible) if a else None,
             physical_traversability=float(a.physical_traversability) if a else None,
             assessment_age_s=age,heading_count=len(node.heading_candidates),traversable_headings=len(headings),
             max_heading_utility=max((float(h.predicted_utility) for h in headings),default=None),
             low_confidence_veto=bool(legacy_ok and hard_heading and conf<.7),eligible=current_ok))
    aggregate.update(counts)
    last_sample=ts
    filename=ROOT/'cdr'/f'{len(snapshots):03d}.cdr'
    payload=bytes(serialize_message(combined)); filename.write_bytes(payload)
    snapshot=dict(index=len(snapshots),received_ns=ts,elapsed_s=(ts-query_received)/1e9,
        source_stamp_ns=ns(combined),origin=origin,current_node_idx=combined.current_node_idx,
        counts=dict(counts),nodes=nodes,eligible_indices=eligible,graph_file=filename.name,
        graph_sha256=hashlib.sha256(payload).hexdigest())
    snapshots.append(snapshot)
    commands.append((ts,1,'P '+' '.join(map(str,[ts,filename,len(eligible),*eligible]))))
    print('snapshot',len(snapshots),'elapsed',snapshot['elapsed_s'],dict(counts),flush=True)

command_text='C\n'+'\n'.join(c for _,_,c in sorted(commands))+'\n'
(ROOT/'replay_commands.txt').write_text(command_text)
result=subprocess.run([str(ROOT/'build/backoff_replay')],input=command_text,text=True,capture_output=True,check=True)
(ROOT/'cpp_replay.log').write_text(result.stdout)
(ROOT/'cpp_replay_stderr.log').write_text(result.stderr)
bytime={s['received_ns']:s for s in snapshots}
for line in result.stdout.splitlines():
    parts=line.split()
    if not parts or parts[0]!='P': continue
    snapshot=bytime[int(parts[1])]; values=[list(map(int,p.split(':'))) for p in parts[3:]]
    snapshot['active_rejected_edges']=int(parts[2]); snapshot['cpp_plans']=values
    snapshot['replay_counts']=dict(eligible=len(values),region_blocked=sum(v[1] for v in values),
        known_before_edge_backoff=sum(v[3] for v in values),known_after_edge_backoff=sum(v[5] for v in values),
        disconnected_by_edge_backoff=sum(v[3] and not v[5] for v in values),
        remaining_known_candidates=sum(not v[1] and v[5] for v in values))
identity=json.loads(Path('/home/ly/wildos-console/source_identity.json').read_text())
summary=dict(scope='Closed recorded topology and score joins; actual production C++ Backoff/Planner. Odom/rejection receipt reconstruction, not a private callback-state capture or physically executed counterfactual.',
    session=str(SESSION),production_source_sha256=identity['sha256'],
    queries=len(data['/spot1/object_search/query']),matching_topology_samples=len(snapshots),
    unmatched_topology_samples=missing,aggregate_node_frame_counts=dict(aggregate),
    accepted_rejections=rejections,snapshots=snapshots,
    production_library_sha256=hashlib.sha256(Path('/home/ly/frontier_cost_repair_20261016/install/lib/libgraphnav_planner_component.so').read_bytes()).hexdigest())
(ROOT/'closed_audit.json').write_text(json.dumps(summary,indent=2))
print(json.dumps({k:v for k,v in summary.items() if k not in ('snapshots','accepted_rejections')},indent=2))
print('SNAPSHOTS',len(snapshots))
print(json.dumps([{k:s[k] for k in ('elapsed_s','counts','replay_counts','active_rejected_edges')} for s in snapshots],indent=2))
