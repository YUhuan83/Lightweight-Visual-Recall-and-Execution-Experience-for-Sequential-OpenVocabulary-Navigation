from pathlib import Path
import ast,hashlib,json,zipfile
repo=Path(r'C:\Users\Yu\Desktop\拟真');root=repo/'test_artifacts/approach_local_map_20261027';c=root/'candidate'
base=json.loads((root/'base_hashes.json').read_text(encoding='utf8'));changes={}
for p in c.rglob('*'):
 if not p.is_file() or '__pycache__' in p.parts or 'test_artifacts' in p.relative_to(c).parts:continue
 name=p.relative_to(c).as_posix();h=hashlib.sha256(p.read_bytes()).hexdigest()
 if base.get(name)==h:continue
 if name.endswith('.py'):ast.parse(p.read_text(encoding='utf8'))
 changes[name]=h
assert len(changes)==2,changes
with zipfile.ZipFile(root/'approach_local_map_delta.zip','w',zipfile.ZIP_DEFLATED) as z:
 z.writestr('base_hashes.json',json.dumps({n:base[n] for n in changes if n in base}))
 for n in changes:z.write(c/n,n)
(root/'source_scope.json').write_text(json.dumps(dict(base='v51_confirmed_action_handoff',target='v52_approach_local_map',changed=changes,acceptance=False),indent=2),encoding='utf8')
aux=Path(__file__).parent
for old,new in [('prepare_confirmed_action_handoff_remote.py','prepare_approach_local_map_remote.py'),('build_confirmed_action_handoff.sh','build_approach_local_map.sh'),('freeze_confirmed_action_handoff.py','freeze_approach_local_map.py')]:
 text=(aux/old).read_text(encoding='utf8').replace('v51_confirmed_action_handoff','v52_approach_local_map').replace('v50_confirmed_continuation','v51_confirmed_action_handoff').replace('confirmed_action_handoff_delta.zip','approach_local_map_delta.zip').replace('test_continuous_action_handoff_ros','test_approach_local_map_ros').replace('freeze_confirmed_action_handoff.py','freeze_approach_local_map.py')
 (aux/new).write_text(text,encoding='utf8',newline='\n')
for name in ('run_confirmed_progress_trial.py','read_vehicle_observation_live.py','audit_vehicle_observation_closed.py','audit_full_path_closed.py'):
 p=aux/name;text=p.read_text(encoding='utf8').replace("'v51_confirmed_action_handoff')","'v51_confirmed_action_handoff','v52_approach_local_map')");p.write_text(text,encoding='utf8',newline='\n')
print(json.dumps(changes,indent=2))
