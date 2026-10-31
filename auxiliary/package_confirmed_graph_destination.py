from pathlib import Path
import ast,hashlib,json,zipfile
repo=Path(r'C:\Users\Yu\Desktop\拟真');root=repo/'test_artifacts/confirmed_graph_destination_20261027';c=root/'candidate'
base=json.loads((root/'base_hashes.json').read_text(encoding='utf8'));changes={}
for p in c.rglob('*'):
 if not p.is_file() or '__pycache__' in p.parts or 'test_artifacts' in p.relative_to(c).parts:continue
 n=p.relative_to(c).as_posix();h=hashlib.sha256(p.read_bytes()).hexdigest()
 if base.get(n)==h:continue
 if n.endswith('.py'):ast.parse(p.read_text(encoding='utf8'))
 changes[n]=h
assert len(changes)==4,changes
with zipfile.ZipFile(root/'confirmed_graph_destination_delta.zip','w',zipfile.ZIP_DEFLATED) as z:
 z.writestr('base_hashes.json',json.dumps({n:base[n] for n in changes if n in base}))
 for n in changes:z.write(c/n,n)
(root/'source_scope.json').write_text(json.dumps(dict(base='v52_approach_local_map',target='v53_confirmed_graph_destination',changed=changes,acceptance=False),indent=2),encoding='utf8')
aux=Path(__file__).parent
for old,new in [('prepare_approach_local_map_remote.py','prepare_confirmed_graph_destination_remote.py'),('build_approach_local_map.sh','build_confirmed_graph_destination.sh'),('freeze_approach_local_map.py','freeze_confirmed_graph_destination.py')]:
 s=(aux/old).read_text(encoding='utf8').replace('v52_approach_local_map','v53_confirmed_graph_destination').replace('v51_confirmed_action_handoff','v52_approach_local_map').replace('approach_local_map_delta.zip','confirmed_graph_destination_delta.zip').replace('test_approach_local_map_ros','test_confirmed_graph_destination_ros').replace('freeze_approach_local_map.py','freeze_confirmed_graph_destination.py')
 (aux/new).write_text(s,encoding='utf8',newline='\n')
for n in ('run_confirmed_progress_trial.py','read_vehicle_observation_live.py','audit_vehicle_observation_closed.py','audit_full_path_closed.py'):
 p=aux/n;s=p.read_text(encoding='utf8').replace("'v52_approach_local_map')","'v52_approach_local_map','v53_confirmed_graph_destination')");p.write_text(s,encoding='utf8',newline='\n')
for old,new in [('audit_confirmed_action_closed.sh','audit_approach_local_closed.sh')]:
 p=aux/old;(aux/new).write_text(p.read_text().replace('v51_confirmed_action_handoff','v52_approach_local_map'),encoding='utf8',newline='\n')
print(json.dumps(changes,indent=2))
