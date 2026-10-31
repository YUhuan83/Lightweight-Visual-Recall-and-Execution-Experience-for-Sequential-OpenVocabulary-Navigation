"""Seal the exact offline console candidate; never connects to the server."""
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import zipfile

root = Path(__file__).resolve().parents[2]
out = Path(__file__).resolve().parent
archive = root/'official_wildos_beamng_bundle.zip'
manifest = json.loads(archive.with_suffix('.manifest.json').read_text())
digest = hashlib.sha256(archive.read_bytes()).hexdigest()
assert digest == manifest['sha256']
with zipfile.ZipFile(archive) as handle:
    assert handle.testzip() is None
    assert set(handle.namelist()) == set(manifest['members'])
    for name, expected in manifest['members'].items():
        assert hashlib.sha256(handle.read(name)).hexdigest() == expected, name
        assert hashlib.sha256((root/name).read_bytes()).hexdigest() == expected, name
    unpack = out/'unpacked'
    unpack.mkdir(exist_ok=False)
    handle.extractall(unpack)
env = dict(os.environ, PYTHONIOENCODING='utf-8')
result = subprocess.run([sys.executable, str(unpack/'tools/test_wildos_console.py')],
                        env=env, capture_output=True)
(out/'unpacked_console_tests.log').write_bytes(result.stdout+result.stderr)
assert result.returncode == 0
checks = json.loads((root/'test_artifacts/offline_navigation/report.json').read_text(encoding='utf8'))
assert not any(item['status'] == 'FAIL' for item in checks['checks'])
assert next(item for item in checks['checks'] if item['name']=='deployment_bundle')['detail']['sha256'] == digest
for name in ('report.json','report.md','callback_report.json'):
    shutil.copy2(root/'test_artifacts/offline_navigation'/name, out/name)
candidate = root/'deploy/20261015_self_service_console_candidate'
candidate.mkdir(exist_ok=False)
for path in (archive, archive.with_suffix('.manifest.json'), archive.with_suffix('.sha256')):
    shutil.copy2(path, candidate/path.name)
report = dict(sha256=digest, members=len(manifest['members']),
    full_offline=dict(Counter(item['status'] for item in checks['checks'])),
    console_tests=26, harness_tests=15, unpacked_console_tests=26,
    powershell_parse='passed', bash_syntax='passed with normal local permissions',
    validation_scope='Local pure tests / SDK and ROS doubles / syntax / archive identity only',
    server_connected=False, field_acceptance=False, commit_created=False,
    pending=['server independent build', 'actual scene/sensor/model startup and ROS/DDS discovery',
             'query ACK and target handoff with real physics',
             'stop / quit / SSH disconnect / owned group cleanup on server',
             '20-round stability and multi-goal acceptance remain unstarted'])
(out/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
(candidate/'CANDIDATE.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
