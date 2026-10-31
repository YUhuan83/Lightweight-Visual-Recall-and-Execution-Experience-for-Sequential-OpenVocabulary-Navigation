"""Verify exact source bytes, callback/DDS boundaries and the closed field result."""
from pathlib import Path
import hashlib
import json
import zipfile

root = Path(__file__).resolve().parent; repo = root.parents[1]
identity = json.loads((root/'sealed/archive.json').read_text())
archive = root/'sealed/confirmed_recovery_evidence.zip'
assert archive.stat().st_size == identity['size']
assert hashlib.sha256(archive.read_bytes()).hexdigest() == identity['sha256']
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None
    manifest = json.loads(z.read('evidence_manifest.json'))
    for name, item in manifest['files'].items():
        data = z.read(name)
        assert len(data) == item['size'] and hashlib.sha256(data).hexdigest() == item['sha256'], name
    frozen = json.loads(z.read('frozen_source_manifest.json'))['members']
    delta = json.loads(z.read('delta_manifest.json'))
    changed = []
    for name, digest in frozen.items():
        assert hashlib.sha256(z.read('candidate/'+name)).hexdigest() == digest, name
        assert hashlib.sha256((repo/name).read_bytes()).hexdigest() == digest, name
        if delta['parent_manifest']['members'].get(name) != digest: changed.append(name)
    assert sorted(changed) == sorted(delta['delta']) and len(changed) == 7
    for name in delta['production_changed_files']:
        assert hashlib.sha256(z.read('before/'+name)).hexdigest() == delta['parent_manifest']['members'][name]
    real = json.loads(z.read('real_ros.json')); assert real['passed'] and real['tests'] == 136
    dds_paths = [n for n in z.namelist() if n.startswith('dds_') and n.endswith('/result.json')]
    dds = [json.loads(z.read(n)) for n in dds_paths]
    assert any(d['passed'] and len(d['stages']) == 13 for d in dds)
    old = json.loads(z.read('closed_before_v2.json')); new = json.loads(z.read('closed_after_v2.json'))
    assert not old['navigation_handoff'] and old['expected'] == old['held_goal']
    assert new['navigation_handoff'] and new['release_pending'] and new['expected'] is None
    assert old['source_failure'] == new['source_failure']
    trial = 'full/trial_20261020_224538'
    result = json.loads(z.read(trial+'/result.json'))
    assert result['closed'] and not result['errors'] and not result['session_result']['errors']
    assert result['goal_result']['state'] == 'timeout' and not result['harness_arrival']
    audit = json.loads(z.read(trial+'/closed_audit.json'))
    assert 166 < audit['traveled_distance_m'] < 168 and 42 < audit['net_displacement_m'] < 43
    visual = json.loads(z.read('field_visual_summary.json'))
    assert visual['query_encoded'] and visual['no_object_after_query'] > 0
    for topic in ('/spot1/object_mask', '/spot1/object_localization', '/spot1/object_confirmation'):
        assert visual['topic_counts'][topic] == 0
    session = trial+'/sessions/session_20261020_224538'
    sources = json.loads(z.read(session+'/verified_sources.json')); assert len(sources) == 108
    for name in delta['production_changed_files']: assert sources[name] == frozen[name]
    stop = json.loads(z.read(session+'/goal_001/stop.json'))
    assert len(stop['samples']) >= 5 and all(s['speed'] < .03 for s in stop['samples'])
    physics = stop['physics']
    assert physics['nodeCount'] == 723 and physics['bodyCollisionNodeCount'] == 467
    assert physics['wheelCount'] == 4 and all(w['collisionNodeCount'] == 32 for w in physics['wheels'])
    assert not physics['collisionOverrideLoaded'] and not physics['configKinematic']
    assert not json.loads(z.read('poststart.json'))['processes']
    (root/'failure_trial224538.png').write_bytes(z.read(session+'/goal_001/failure_last_rgb.png'))
offline = json.loads((repo/'test_artifacts/offline_navigation/callback_report.json').read_text())
assert offline['passed'] and offline['tests'] == 608
before = json.loads((root/'before.json').read_text())
assert (before['tests'], before['failures'], before['errors']) == (9,9,0)
report = dict(verified=True, source_members=len(frozen), changed_files=changed,
    offline_tests=608, real_ros_callback_tests=136, dds_stages=13,
    closed_recorded_handoff_before=False, closed_recorded_handoff_after=True,
    field_result='timeout', approach_recovery_exercised_in_field=False,
    complete_navigation_passed=False, archive_sha256=identity['sha256'])
(root/'local_verification.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
