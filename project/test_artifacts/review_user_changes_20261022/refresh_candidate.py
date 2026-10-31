"""Update reviewed source/test bytes before any field process has started."""
from pathlib import Path
import hashlib
import json
import zipfile

ROOT = Path('/home/ly/execution_continuity_review_20261022')
dest = ROOT / 'v1'
assert not list((dest / 'full').glob('trial_*'))
plan = json.loads((ROOT / 'incoming/deployment_plan.json').read_text())
archive = ROOT / 'incoming/candidate_delta.zip'
assert hashlib.sha256(archive.read_bytes()).hexdigest() == plan['archive_sha256']
frozen = json.loads((dest / 'full/frozen_source_manifest.json').read_text())
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None
    for name, expected in plan['members'].items():
        data = z.read(name)
        assert hashlib.sha256(data).hexdigest() == expected
        target = dest / 'candidate' / name
        assert target.resolve().is_relative_to((dest / 'candidate').resolve())
        target.write_bytes(data)
        frozen['members'][name] = expected
(dest / 'full/frozen_source_manifest.json').write_text(json.dumps(frozen, indent=2))
(dest / 'deployment_receipt.json').write_text(json.dumps(plan, indent=2))
print(json.dumps({'updated': len(plan['members']), 'field_started': False}))
