import ast,difflib,hashlib,json,zipfile
from pathlib import Path
root=Path(__file__).parent.parent/'拟真/test_artifacts/full_path_control_20261026'
candidate=root/'candidate';base=json.loads((root/'base_hashes.json').read_text(encoding='utf8'))
members={p.relative_to(candidate).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
         for p in candidate.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
changed=[n for n,h in members.items() if base.get(n)!=h and not n.startswith('test_artifacts/')]
for n in changed:
    if n.endswith('.py'):ast.parse((candidate/n).read_text(encoding='utf8'))
with zipfile.ZipFile(root/'full_path_delta.zip','w',zipfile.ZIP_DEFLATED) as z:
    z.writestr('base_hashes.json',json.dumps({n:base[n] for n in changed if n in base},indent=2))
    for n in changed:z.write(candidate/n,n)
report=dict(base='v35_confirmed_vehicle',target='v37_full_path',changed={n:members[n] for n in changed},
    status='candidate; no field acceptance; no main runtime overwrite; no commit')
(root/'source_scope.json').write_text(json.dumps(report,indent=2),encoding='utf8')
print(json.dumps(report,indent=2))
