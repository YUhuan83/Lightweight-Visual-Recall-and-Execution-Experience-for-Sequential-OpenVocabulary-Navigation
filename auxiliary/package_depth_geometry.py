import ast,hashlib,json,zipfile
from pathlib import Path
repo=Path(__file__).parent.parent/'拟真';root=repo/'test_artifacts/depth_geometry_20261027';candidate=root/'candidate'
base=json.loads((root/'base_hashes.json').read_text(encoding='utf8'));changed={}
for p in candidate.rglob('*'):
    if not p.is_file() or '__pycache__' in p.parts:continue
    name=p.relative_to(candidate).as_posix()
    if name.startswith('test_artifacts/'):continue
    h=hashlib.sha256(p.read_bytes()).hexdigest()
    if h==base.get(name):continue
    if name.endswith('.py'):ast.parse(p.read_text(encoding='utf8'))
    changed[name]=h
assert set(changed)=={'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/explorfm_triangulation/localization_geometry.py', 'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/test/fixtures/scanline_depth_v46.npz', 'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/test/test_scanline_depth.py'},changed
with zipfile.ZipFile(root/'depth_geometry_delta.zip','w',zipfile.ZIP_DEFLATED) as z:
    z.writestr('base_hashes.json',json.dumps({n:base[n] for n in changed if n in base},indent=2))
    for n in changed:z.write(candidate/n,n)
(root/'source_scope.json').write_text(json.dumps(dict(base='v46_approach_feedback',target='v47_depth_geometry',changed=changed,acceptance=False),indent=2),encoding='utf8')
print(json.dumps(changed,indent=2))
