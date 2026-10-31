import ast,hashlib,json,zipfile
from pathlib import Path
repo=Path(__file__).parent.parent/'拟真';root=repo/'test_artifacts/retirement_boundary_20261026';candidate=root/'candidate'
base=json.loads((root/'base_hashes.json').read_text(encoding='utf8'));changed={}
for p in candidate.rglob('*'):
    if not p.is_file() or '__pycache__' in p.parts:continue
    name=p.relative_to(candidate).as_posix()
    if name.startswith('test_artifacts/'):continue
    h=hashlib.sha256(p.read_bytes()).hexdigest()
    if h==base.get(name):continue
    if name.endswith('.py'):ast.parse(p.read_text(encoding='utf8'))
    changed[name]=h
assert set(changed)=={'nav2_adapter/wildos_nav2_adapter/continuous_graph_path_bridge.py',
 'nav2_adapter/test/test_retirement_boundary_ros.py','ackermann_rpp/src/controller.cpp',
 'ackermann_rpp/test/test_controller.cpp','ackermann_rpp/test/retirement_cases.inc'},changed
with zipfile.ZipFile(root/'retirement_boundary_delta.zip','w',zipfile.ZIP_DEFLATED) as z:
    z.writestr('base_hashes.json',json.dumps({n:base[n] for n in changed if n in base},indent=2))
    for n in changed:z.write(candidate/n,n)
(root/'source_scope.json').write_text(json.dumps(dict(base='v40_observation_heading',target='v42_retirement_boundary',changed=changed,acceptance=False),indent=2),encoding='utf8')
print(json.dumps(changed,indent=2))
