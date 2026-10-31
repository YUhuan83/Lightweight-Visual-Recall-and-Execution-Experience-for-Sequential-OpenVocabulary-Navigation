"""Preserve failed validation; update only reviewed candidate files before freeze."""
import hashlib
import json
from pathlib import Path
import shutil
import zipfile

base = Path('/home/ly/execution_continuity_review_20261022')
root = base/'v46_approach_feedback'
parent = base/'v45_approach_contract'
assert not (root/'field_result.json').exists()
assert not (root/'trial_frozen.json').exists()
log = root/'real_ros_review_1.log'
assert log.exists() and not (root/'initial_missing_anchor_fixture_failure.log').exists()
shutil.copy2(log, root/'initial_missing_anchor_fixture_failure.log')
with zipfile.ZipFile(base/'approach_feedback_delta.zip') as z:
    expected = json.loads(z.read('base_hashes.json'))
    for name, digest in expected.items():
        assert hashlib.sha256((parent/'candidate'/name).read_bytes()).hexdigest() == digest
    manifest_path = root/'full/frozen_source_manifest.json'
    manifest = json.loads(manifest_path.read_text())
    for name in z.namelist():
        if name == 'base_hashes.json':
            continue
        current = root/'candidate'/name
        assert hashlib.sha256(current.read_bytes()).hexdigest() == manifest['members'][name]
        current.write_bytes(z.read(name))
        manifest['members'][name] = hashlib.sha256(current.read_bytes()).hexdigest()
    manifest_path.write_text(json.dumps(manifest, indent=2))
print('Updated reviewed candidate; initial failed validation retained.')
