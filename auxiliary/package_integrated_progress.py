import hashlib,json,zipfile
from pathlib import Path
root=Path(__file__).parent.parent/'拟真/test_artifacts/integrated_progress_20261025';candidate=root/'candidate'
base=json.loads((root/'base_hashes.json').read_text())
all_names={p.relative_to(candidate).as_posix() for p in candidate.rglob('*') if p.is_file() and '__pycache__' not in p.parts and not p.relative_to(candidate).as_posix().startswith('test_artifacts/offline_navigation/')}
changed=sorted(n for n in all_names if n not in base or hashlib.sha256((candidate/n).read_bytes()).hexdigest()!=base[n])
runtime=[n for n in changed if '/test/' not in n]
assert set(runtime)=={'official_adapter/launch/official_graph_stack.launch.py','wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/explorfm_triangulation/localization_geometry.py','wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/explorfm_triangulation/obj_mask_triangulation.py'},runtime
with zipfile.ZipFile(root/'integrated_progress_delta.zip','w',zipfile.ZIP_DEFLATED) as z:
    z.writestr('base_hashes.json',json.dumps({n:base[n] for n in changed if n in base},indent=2))
    for n in changed:z.write(candidate/n,n)
report=dict(base='v32_retry_admission',runtime=runtime,changed={n:hashlib.sha256((candidate/n).read_bytes()).hexdigest() for n in changed},scope='Merge separately validated v29r2 geometry and v30 confirmation history, retain v31/v32 contracts')
(root/'source_scope.json').write_text(json.dumps(report,indent=2));print(json.dumps(dict(files=len(changed),runtime=runtime)))
