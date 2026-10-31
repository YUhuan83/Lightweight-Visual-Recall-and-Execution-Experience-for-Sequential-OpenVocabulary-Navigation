from pathlib import Path
import ast,hashlib,json,zipfile
repo=Path(r'C:\Users\Yu\Desktop\拟真');root=repo/'test_artifacts/localization_range_20261027';c=root/'candidate';base=json.loads((root/'base_hashes.json').read_text(encoding='utf8'));changes={}
for p in c.rglob('*'):
 if not p.is_file() or '__pycache__' in p.parts or 'test_artifacts' in p.relative_to(c).parts:continue
 n=p.relative_to(c).as_posix();h=hashlib.sha256(p.read_bytes()).hexdigest()
 if h==base.get(n):continue
 if n.endswith('.py'):ast.parse(p.read_text(encoding='utf8'))
 changes[n]=h
assert len(changes)==8,changes
with zipfile.ZipFile(root/'localization_range_delta.zip','w',zipfile.ZIP_DEFLATED) as z:
 z.writestr('base_hashes.json',json.dumps({n:base[n] for n in changes if n in base}))
 for n in changes:z.write(c/n,n)
(root/'source_scope.json').write_text(json.dumps(dict(base='v48_range_candidate',target='v49_localization_range',changed=changes,acceptance=False),indent=2),encoding='utf8')
aux=Path(__file__).parent
s=(aux/'prepare_observation_heading_remote.py').read_text(encoding='utf8').replace("source=base/'v39_observation_session';target=base/'v40_observation_heading'","source=base/'v48_range_candidate';target=base/'v49_localization_range'").replace('observation_heading_delta.zip','localization_range_delta.zip').replace('test_recovery_heading_ros','test_localization_range_ros')
(aux/'prepare_localization_range_remote.py').write_text(s,encoding='utf8',newline='\n')
s=(aux/'build_range_candidate.sh').read_text(encoding='utf8').replace('v48_range_candidate','v49_localization_range').replace('freeze_range_candidate.py','freeze_localization_range.py')
s=s.replace('python candidate/tools/test_wildos_console.py','python candidate/tools/test_physical_sensor_rig.py >rig_test.log 2>&1\npython candidate/tools/test_wildos_console.py')
(aux/'build_localization_range.sh').write_text(s,encoding='utf8',newline='\n')
s=(aux/'freeze_range_candidate.py').read_text(encoding='utf8').replace('v48_range_candidate','v49_localization_range').replace('base="v47_depth_geometry"','base="v48_range_candidate"').replace('Current accepted range guides bounded observation before consensus; exact-mask identity and original final confirmation preserved; field pending','80m measured localization stream separated from 40m navigation with independent point budgets; existing frames, timestamps, quality and physical controls preserved; field pending')
(aux/'freeze_localization_range.py').write_text(s,encoding='utf8',newline='\n')
print(json.dumps(changes,indent=2))
