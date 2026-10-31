"""Isolated source candidate; preserve all prior installations and evidence."""
from pathlib import Path
import hashlib
import json
import shutil

root = Path(__file__).resolve().parent
spec = json.loads((root/'delta_manifest.json').read_text())
parent = Path('/home/ly/graph_start_attachment_repair_20261020/candidate')
members = spec['parent_manifest']['members']
assert all(hashlib.sha256((parent/n).read_bytes()).hexdigest() == h for n, h in members.items())
candidate = root/'candidate'
candidate.mkdir(exist_ok=False)
for n in members:
    p = candidate/n; p.parent.mkdir(parents=True, exist_ok=True); shutil.copy2(parent/n, p)
for n, h in spec['delta'].items():
    source = root/'delta'/n
    assert hashlib.sha256(source.read_bytes()).hexdigest() == h, n
    p = candidate/n; p.parent.mkdir(parents=True, exist_ok=True); shutil.copy2(source, p)
    members[n] = h
for p in (root/'before').rglob('*.py'):
    n = str(p.relative_to(root/'before'))
    if n in spec['parent_manifest']['members']:
        assert p.read_bytes() == (parent/n).read_bytes(), n
helpers = candidate/'test_artifacts/confirmed_approach_recovery_20261020'
helpers.mkdir(parents=True)
shutil.copy2(root/'run_callbacks.py', helpers/'run_callbacks.py')
(root/'frozen_source_manifest.json').write_text(json.dumps(dict(members=members), indent=2)+'\n')
print(json.dumps(dict(source_members=len(members), changed=len(spec['delta']), staged=True)))
