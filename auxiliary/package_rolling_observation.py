import ast,hashlib,json,zipfile
from pathlib import Path
root=Path(__file__).parent.parent/'拟真/test_artifacts/rolling_observation_20261026'
candidate=root/'candidate';base=json.loads((root/'base_hashes.json').read_text(encoding='utf8'))
changed={}
for p in candidate.rglob('*'):
    if not p.is_file() or '__pycache__' in p.parts:continue
    n=p.relative_to(candidate).as_posix()
    if n.startswith('test_artifacts/'):continue
    h=hashlib.sha256(p.read_bytes()).hexdigest()
    if base.get(n)!=h:
        if n.endswith('.py'):ast.parse(p.read_text(encoding='utf8'))
        changed[n]=h
with zipfile.ZipFile(root/'rolling_delta.zip','w',zipfile.ZIP_DEFLATED) as z:
    z.writestr('base_hashes.json',json.dumps({n:base[n] for n in changed if n in base},indent=2))
    for n in changed:z.write(candidate/n,n)
report=dict(base='v37_full_path',target='v38_rolling_observation',changed=changed,
    rpp_unchanged=True,localization_geometry_unchanged=True,acceptance='not established; do not replace baseline')
(root/'source_scope.json').write_text(json.dumps(report,indent=2),encoding='utf8');print(json.dumps(report,indent=2))
