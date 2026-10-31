from pathlib import Path
import ast, hashlib, json, zipfile
repo=Path(r'C:\Users\Yu\Desktop\拟真'); root=repo/'test_artifacts/confirmed_continuation_20261027'; c=root/'candidate'
base=json.loads((root/'base_hashes.json').read_text(encoding='utf8')); changes={}
for p in c.rglob('*'):
    if not p.is_file() or '__pycache__' in p.parts or 'test_artifacts' in p.relative_to(c).parts: continue
    name=p.relative_to(c).as_posix(); digest=hashlib.sha256(p.read_bytes()).hexdigest()
    if base.get(name)==digest: continue
    if name.endswith('.py'): ast.parse(p.read_text(encoding='utf8'))
    changes[name]=digest
assert len(changes)==3, changes
with zipfile.ZipFile(root/'confirmed_continuation_delta.zip','w',zipfile.ZIP_DEFLATED) as z:
    z.writestr('base_hashes.json',json.dumps({n:base[n] for n in changes if n in base}))
    for n in changes: z.write(c/n,n)
(root/'source_scope.json').write_text(json.dumps(dict(base='v49_localization_range',target='v50_confirmed_continuation',changed=changes,acceptance=False),indent=2),encoding='utf8')
aux=Path(__file__).parent
text=(aux/'prepare_localization_range_remote.py').read_text(encoding='utf8').replace("source=base/'v48_range_candidate';target=base/'v49_localization_range'", "source=base/'v49_localization_range';target=base/'v50_confirmed_continuation'").replace('localization_range_delta.zip','confirmed_continuation_delta.zip').replace('test_localization_range_ros','test_confirmed_continuation_ros')
(aux/'prepare_confirmed_continuation_remote.py').write_text(text,encoding='utf8',newline='\n')
text=(aux/'build_localization_range.sh').read_text(encoding='utf8').replace('v49_localization_range','v50_confirmed_continuation').replace('freeze_localization_range.py','freeze_confirmed_continuation.py')
(aux/'build_confirmed_continuation.sh').write_text(text,encoding='utf8',newline='\n')
text=(aux/'freeze_localization_range.py').read_text(encoding='utf8').replace('v49_localization_range','v50_confirmed_continuation').replace('base="v48_range_candidate"','base="v49_localization_range"').replace('80m measured localization stream separated from 40m navigation with independent point budgets; existing frames, timestamps, quality and physical controls preserved; field pending','Same confirmed target updates the route through the existing moving intent handoff; no new execution state or relaxed arrival, collision or reversing checks; field pending')
(aux/'freeze_confirmed_continuation.py').write_text(text,encoding='utf8',newline='\n')
for name in ('run_confirmed_progress_trial.py','read_vehicle_observation_live.py','audit_vehicle_observation_closed.py','audit_full_path_closed.py','audit_retirement_boundary_closed.py'):
    p=aux/name; text=p.read_text(encoding='utf8')
    if "'v50_confirmed_continuation'" not in text:
        text=text.replace("'v49_localization_range'", "'v49_localization_range','v50_confirmed_continuation'", 1)
        p.write_text(text,encoding='utf8',newline='\n')
print(json.dumps(changes,indent=2))
