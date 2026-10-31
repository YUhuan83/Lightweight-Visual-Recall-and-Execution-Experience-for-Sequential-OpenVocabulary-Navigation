import hashlib,json,shlex,subprocess,sys
from pathlib import Path
from rclpy.serialization import deserialize_message,serialize_message
from graphnav_msgs.msg import NavigationGraph
root=Path('/home/ly/execution_continuity_review_20261022/v71_heading_freshness')
out=root/'target_frontier_trace';out.mkdir(exist_ok=True)
trial=root/'full/trial_20261030_174941';assert json.loads((trial/'result.json').read_text())['closed']
folder=next(trial.glob('sessions/*/graph_pairs'))
rows=[json.loads(x) for x in (folder/'snapshots.jsonl').read_text().splitlines()]
pairs=[r for r in rows if r.get('kind')=='pair' and all(r.get('contract',{}).values())]
plans=json.loads((trial/'target_path_interface_audit.json').read_text())['plans']
cases=[]
for i,plan in enumerate(plans[:3]):
    stamp=plan['route_stamp_ns'];pair=max((r for r in pairs if r['ros_ns']<=stamp),key=lambda r:r['ros_ns'])
    raw=deserialize_message((folder/pair['raw']['file']).read_bytes(),NavigationGraph)
    scores=deserialize_message((folder/pair['scored']['file']).read_bytes(),NavigationGraph)
    raw.nodes=scores.nodes;filename=out/f'case_{i}.cdr';filename.write_bytes(serialize_message(raw))
    goal=plan['graph_points'][-1]
    (out/f'case_{i}_edges.json').write_text(json.dumps([dict(
        source=e.from_idx,target=e.to_idx,
        assessments=[dict(observed_ns=a.observed_at.sec*10**9+a.observed_at.nanosec,
            age_now_s=(stamp-a.observed_at.sec*10**9-a.observed_at.nanosec)*1e-9,
            age_graph_s=(raw.header.stamp.sec*10**9+raw.header.stamp.nanosec-a.observed_at.sec*10**9-a.observed_at.nanosec)*1e-9,
            confidence=a.confidence,trav=a.physical_traversability) for a in e.robot_traversability],
        properties={p.key:list(p.value) for p in e.properties}) for e in raw.edges],indent=2))
    cases.append(dict(file=str(filename),goal=goal,stamp=stamp,pair=pair['scored']['file'],
        pair_age_s=(stamp-pair['ros_ns'])*1e-9,plan=plan,
        nodes=[dict(index=j,xyz=[n.pose.position.x,n.pose.position.y,n.pose.position.z],
                    headings=[dict(heading=h.heading,traversable=h.traversable,utility=h.predicted_utility) for h in n.heading_candidates],
                    assessments=[dict(age_s=(stamp-a.observed_at.sec*10**9-a.observed_at.nanosec)*1e-9,
                        confidence=a.confidence,feasible=a.feasible,trav=a.physical_traversability,risk=a.dynamic_risk) for a in n.robot_traversability],
                    properties={p.key:list(p.value) for p in n.properties}) for j,n in enumerate(raw.nodes)]))
(out/'scope.json').write_text(json.dumps(cases,indent=2))
if '--prepare-only' in sys.argv:sys.exit(0)
build=root/'build/graphnav_planner';flags={}
for line in (build/'CMakeFiles/test_frontier_endpoint.dir/flags.make').read_text().splitlines():
    if ' = ' in line:
        k,v=line.split(' = ',1);flags[k]=shlex.split(v)
obj=out/'trace.o';exe=out/'trace'
subprocess.run(['/usr/bin/c++',*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],
    '-c',str(root/'trace_frontier.cpp'),'-o',str(obj)],cwd=build,check=True)
link=shlex.split((build/'CMakeFiles/test_frontier_endpoint.dir/link.txt').read_text())
link[1]=str(obj);link[link.index('-o')+1]=str(exe)
subprocess.run(link,cwd=build,check=True)
for i,case in enumerate(cases):
    for mode in ('baseline','edge_time_now'):
        log=out/f'case_{i}_{mode}.log'
        with log.open('w') as f:
            subprocess.run([str(exe),case['file'],*map(str,case['goal']),str(case['stamp']),mode],stdout=f,stderr=subprocess.STDOUT,check=True)
        print('case',i,'age',case['pair_age_s'],mode, next(x for x in log.read_text().splitlines() if x.startswith('PATH')),flush=True)
