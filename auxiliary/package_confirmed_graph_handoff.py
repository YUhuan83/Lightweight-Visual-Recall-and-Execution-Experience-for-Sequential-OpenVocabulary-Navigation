from pathlib import Path
import ast,hashlib,json,zipfile
repo=Path(r'C:\Users\Yu\Desktop\拟真');root=repo/'test_artifacts/confirmed_graph_handoff_20261027';c=root/'candidate'
base=json.loads((root/'base_hashes.json').read_text(encoding='utf8'));changes={}
for p in c.rglob('*'):
 if not p.is_file() or '__pycache__' in p.parts or 'test_artifacts' in p.relative_to(c).parts:continue
 n=p.relative_to(c).as_posix();h=hashlib.sha256(p.read_bytes()).hexdigest()
 if base.get(n)==h:continue
 if n.endswith('.py'):ast.parse(p.read_text(encoding='utf8'))
 changes[n]=h
assert len(changes)==3,changes
with zipfile.ZipFile(root/'confirmed_graph_handoff_delta.zip','w',zipfile.ZIP_DEFLATED) as z:
 z.writestr('base_hashes.json',json.dumps({n:base[n] for n in changes if n in base}))
 for n in changes:z.write(c/n,n)
(root/'source_scope.json').write_text(json.dumps(dict(base='v53_confirmed_graph_destination',target='v54_confirmed_graph_handoff',changed=changes,acceptance=False),indent=2),encoding='utf8')
aux=Path(__file__).parent
for old,new in [('prepare_confirmed_graph_destination_remote.py','prepare_confirmed_graph_handoff_remote.py'),('build_confirmed_graph_destination.sh','build_confirmed_graph_handoff.sh'),('freeze_confirmed_graph_destination.py','freeze_confirmed_graph_handoff.py')]:
 s=(aux/old).read_text(encoding='utf8').replace('v53_confirmed_graph_destination','v54_confirmed_graph_handoff').replace('v52_approach_local_map','v53_confirmed_graph_destination').replace('confirmed_graph_destination_delta.zip','confirmed_graph_handoff_delta.zip').replace('test_confirmed_graph_destination_ros','test_confirmed_graph_handoff_ros').replace('freeze_confirmed_graph_destination.py','freeze_confirmed_graph_handoff.py')
 (aux/new).write_text(s,encoding='utf8',newline='\n')
for n in ('run_confirmed_progress_trial.py','read_vehicle_observation_live.py','audit_vehicle_observation_closed.py','audit_full_path_closed.py'):
 p=aux/n;s=p.read_text(encoding='utf8').replace("'v53_confirmed_graph_destination')","'v53_confirmed_graph_destination','v54_confirmed_graph_handoff')");p.write_text(s,encoding='utf8',newline='\n')
(aux/'audit_confirmed_graph_destination_closed.sh').write_text((aux/'audit_approach_local_closed.sh').read_text().replace('v52_approach_local_map','v53_confirmed_graph_destination'),encoding='utf8',newline='\n')
print(json.dumps(changes,indent=2))
