from pathlib import Path
import ast,hashlib,json,zipfile
repo=Path(r'C:\Users\Yu\Desktop\拟真');root=repo/'test_artifacts/range_candidate_20261027';c=root/'candidate';base=json.loads((root/'base_hashes.json').read_text(encoding='utf8'));changes={}
for p in c.rglob('*'):
 if not p.is_file() or '__pycache__' in p.parts or 'test_artifacts' in p.relative_to(c).parts:continue
 n=p.relative_to(c).as_posix();h=hashlib.sha256(p.read_bytes()).hexdigest()
 if h==base.get(n):continue
 if n.endswith('.py'):ast.parse(p.read_text(encoding='utf8'))
 changes[n]=h
assert set(changes)=={'nav2_adapter/test/test_range_candidate_ros.py','nav2_adapter/wildos_nav2_adapter/visible_approach.py','official_adapter/beamng_wildos_adapter/goal_standoff.py'},changes
with zipfile.ZipFile(root/'range_candidate_delta.zip','w',zipfile.ZIP_DEFLATED) as z:
 z.writestr('base_hashes.json',json.dumps({n:base[n] for n in changes if n in base}))
 for n in changes:z.write(c/n,n)
(root/'source_scope.json').write_text(json.dumps(dict(base='v47_depth_geometry',target='v48_range_candidate',changed=changes,acceptance=False),indent=2),encoding='utf8')
aux=Path(__file__).parent
s=(aux/'prepare_observation_heading_remote.py').read_text(encoding='utf8').replace("source=base/'v39_observation_session';target=base/'v40_observation_heading'","source=base/'v47_depth_geometry';target=base/'v48_range_candidate'").replace('observation_heading_delta.zip','range_candidate_delta.zip').replace('test_recovery_heading_ros','test_range_candidate_ros')
(aux/'prepare_range_candidate_remote.py').write_text(s,encoding='utf8',newline='\n')
s=(aux/'build_depth_geometry.sh').read_text(encoding='utf8').replace('v47_depth_geometry','v48_range_candidate').replace('freeze_depth_geometry.py','freeze_range_candidate.py')
(aux/'build_range_candidate.sh').write_text(s,encoding='utf8',newline='\n')
s=(aux/'freeze_depth_geometry.py').read_text(encoding='utf8').replace('v47_depth_geometry','v48_range_candidate').replace('v46_approach_feedback','v47_depth_geometry').replace('Independent measured LiDAR scanline position separated from plane normal observability; quality/arrival safety and vehicle unchanged; full acceptance pending','Current accepted range guides bounded observation before consensus; exact-mask identity and original final confirmation preserved; field pending')
(aux/'freeze_range_candidate.py').write_text(s,encoding='utf8',newline='\n')
print(json.dumps(changes,indent=2))
