"""Verify sealed diagnostic evidence, not task arrival."""
from pathlib import Path
import hashlib
import json
import zipfile

root = Path(__file__).resolve().parent
archive = root/'idle_connectivity_evidence.zip'
digest = hashlib.sha256(archive.read_bytes()).hexdigest()
assert digest == '1c20ea28601c63518c0af2930eba98d24cf6bf60c1bd5d6fcce5060328c3bb8b'
with zipfile.ZipFile(archive) as zf:
    assert zf.testzip() is None
    manifest = json.loads(zf.read('manifest.json'))
    assert len(manifest) == 16
    for name, want in manifest.items():
        payload = zf.read(name)
        assert len(payload) == want['bytes']
        assert hashlib.sha256(payload).hexdigest() == want['sha256'], name
    idle = json.loads(zf.read('session/idle_connectivity_review.json'))
    stop = json.loads(zf.read('session/stop.json'))
    assert idle['goals'] == idle['positive_drive_commands'] == idle['nonzero_speed_targets'] == 0
    assert idle['controller_reasons'] == ['disabled']
    assert idle['maximum_observed_speed'] < .05
    assert idle['navigation_success'] is False and idle['historical_latency_fixed'] is False
    assert idle['errors'] == []
    for name in ('idle_connectivity_review.json', 'udp_timing_review.json', 'stop.json', 'session_result.json'):
        (root/name).write_bytes(zf.read('session/'+name))
report = dict(archive_sha256=digest, payloads=16, crc_and_all_hashes_verified=True,
              brake_only_verified=True, navigation_success=False,
              historical_latency_fixed=False)
(root/'idle_verification.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
