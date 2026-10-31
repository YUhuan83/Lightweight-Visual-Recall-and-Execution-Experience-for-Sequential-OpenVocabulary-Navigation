import ast,hashlib,json,zipfile
from pathlib import Path
repo=Path(__file__).parent.parent/'拟真';root=repo/'test_artifacts/observation_heading_20261026';candidate=root/'candidate'
base=json.loads((root/'base_hashes.json').read_text(encoding='utf8'));changed={}
for p in candidate.rglob('*'):
    if not p.is_file() or '__pycache__' in p.parts:continue
    name=p.relative_to(candidate).as_posix();h=hashlib.sha256(p.read_bytes()).hexdigest()
    if h==base.get(name):continue
    if name.endswith('.py'):ast.parse(p.read_text(encoding='utf8'))
    changed[name]=h
assert set(changed)=={'nav2_adapter/wildos_nav2_adapter/target_observation.py','nav2_adapter/test/test_recovery_heading_ros.py'},changed
with zipfile.ZipFile(root/'observation_heading_delta.zip','w',zipfile.ZIP_DEFLATED) as z:
    z.writestr('base_hashes.json',json.dumps({n:base[n] for n in changed if n in base},indent=2))
    for n in changed:z.write(candidate/n,n)
(root/'source_scope.json').write_text(json.dumps(dict(base='v39_observation_session',target='v40_observation_heading',changed=changed,acceptance=False),indent=2),encoding='utf8')
print(json.dumps(changed,indent=2))
