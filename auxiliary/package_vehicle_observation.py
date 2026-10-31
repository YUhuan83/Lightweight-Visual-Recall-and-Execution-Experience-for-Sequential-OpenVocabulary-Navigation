import hashlib,json,zipfile
from pathlib import Path
root=Path(__file__).parent.parent/'拟真/test_artifacts/vehicle_observation_repair_20261025';candidate=root/'candidate'
base=json.loads((root/'base_hashes.json').read_text(encoding='utf8'))
names={p.relative_to(candidate).as_posix() for p in candidate.rglob('*') if p.is_file() and '__pycache__' not in p.parts and not p.relative_to(candidate).as_posix().startswith('test_artifacts/')}
changed=sorted(n for n in names if n not in base or hashlib.sha256((candidate/n).read_bytes()).hexdigest()!=base[n])
runtime=[n for n in changed if '/test/' not in n]
assert len(runtime)==7,runtime
with zipfile.ZipFile(root/'vehicle_observation_delta.zip','w',zipfile.ZIP_DEFLATED) as z:
    z.writestr('base_hashes.json',json.dumps({n:base[n] for n in changed if n in base},indent=2))
    for n in changed:z.write(candidate/n,n)
(root/'source_scope.json').write_text(json.dumps(dict(base='v33_integrated_progress',runtime=runtime,changed={n:hashlib.sha256((candidate/n).read_bytes()).hexdigest() for n in changed}),indent=2),encoding='utf8')
print(json.dumps(dict(changed=len(changed),runtime=runtime)))
