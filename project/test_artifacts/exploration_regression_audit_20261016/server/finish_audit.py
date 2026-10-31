from pathlib import Path
import hashlib,json,sqlite3,subprocess,zipfile
from rclpy.serialization import deserialize_message
from std_msgs.msg import String

root=Path('/home/ly/exploration_regression_audit_20261016')
report=json.loads((root/'closed_audit.json').read_text())
session=Path(report['session'])
assert (session/'session_result.json').exists() and (session/'goal_001/result.json').exists()
commands=[]
for line in (root/'replay_commands.txt').read_text().splitlines():
    parts=line.split()
    if parts and parts[0]!='C':commands.append((int(parts[1]),0 if parts[0]=='R' else 1,line))
with sqlite3.connect(next((session/'goal_001/interfaces').glob('*.db3')).as_uri()+'?mode=ro',uri=True) as db:
    tid=db.execute("select id from topics where name='/spot1/graphnav_planner/route_status'").fetchone()[0]
    decisions=[dict(received_ns=t,**json.loads(deserialize_message(b,String).data))
        for t,b in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid,))]
for d in decisions:
    commands.append((d['stamp_ns'],2,'Q '+' '.join(map(str,[d['stamp_ns'],1,*d['goal']]))))
value=subprocess.run([str(root/'build/backoff_replay')],input='C\n'+'\n'.join(c for _,_,c in sorted(commands))+'\n',
    text=True,capture_output=True,check=True)
(root/'cpp_extended_replay.log').write_text(value.stdout)
q=[l.split() for l in value.stdout.splitlines() if l.startswith('Q ')]
assert len(q)==len(decisions)
report['decision_crosscheck']=dict(total=len(q),complete_graph_routes=sum(d['complete'] for d in decisions),
    selected_goal_blocked_in_reconstruction=sum(int(l[2]) for l in q),
    source_interdecision_max_gap_s=max((b['stamp_ns']-a['stamp_ns'])/1e9 for a,b in zip(decisions,decisions[1:])))
report['decisions']=decisions
# Check critical local bundle sources against the frozen source manifest.
identity=json.loads(Path('/home/ly/wildos-console/source_identity.json').read_text())
critical=['nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py',
    'nav2_adapter/wildos_nav2_adapter/execution_recovery.py',
    'nav2_adapter/wildos_nav2_adapter/exploration_cost.py',
    'wildos_port/repro_check/src/nebula2-wildos/graphnav_planner/src/planner_node.cpp',
    'wildos_port/repro_check/src/nebula2-wildos/graphnav_planner/src/planner.cpp',
    'wildos_port/repro_check/src/nebula2-wildos/graphnav_planner/include/graphnav_planner/exploration_backoff.hpp',
    'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/wildos/nav.py']
report['critical_runtime_source_hashes']={}
for name in critical:
    actual=hashlib.sha256((Path('/home/ly/wildos-console/bundle')/name).read_bytes()).hexdigest()
    assert actual==identity['members'][name]
    report['critical_runtime_source_hashes'][name]=actual
(root/'closed_audit.json').write_text(json.dumps(report,indent=2))
kept_cdr={min(report['snapshots'],key=lambda s:abs(s['elapsed_s']-t))['graph_file']
          for t in (0.,240.4,312.9,418.3)}
external={str(p):dict(size=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest())
          for p in (root/'cdr').glob('*.cdr') if p.name not in kept_cdr}
(root/'cdr_external_manifest.json').write_text(json.dumps(external,indent=2))
files=[p for p in root.rglob('*') if p.is_file() and 'build' not in p.relative_to(root).parts
       and p.suffix not in ('.zip',) and p.name not in ('archive_manifest.json','archive_meta.json')
       and (p.suffix!='.cdr' or p.name in kept_cdr)]
assert sum(p.stat().st_size for p in files)<100*1024**2
manifest={str(p.relative_to(root)):dict(size=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in files}
(root/'archive_manifest.json').write_text(json.dumps(manifest,indent=2))
archive=root/'closed_audit_evidence.zip'
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
    for p in files+[root/'archive_manifest.json']:z.write(p,p.relative_to(root))
meta=dict(sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),size=archive.stat().st_size,members=len(files)+1,
    decision_crosscheck=report['decision_crosscheck'])
(root/'archive_meta.json').write_text(json.dumps(meta,indent=2))
print(json.dumps(meta,indent=2))
