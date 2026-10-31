"""Verify closed evidence, current candidate and stated validation boundaries."""
from pathlib import Path
import hashlib
import json
import zipfile

root = Path(__file__).resolve().parent
project = root.parents[1]
identity = json.loads((root/'archive_identity.json').read_text())
archive = root/'execution_evidence.zip'
assert archive.stat().st_size == identity['bytes']
assert hashlib.sha256(archive.read_bytes()).hexdigest() == identity['sha256']
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None
    manifest = json.loads(z.read('evidence_manifest.json'))
    for name, meta in manifest['members'].items():
        data = z.read(name)
        assert len(data) == meta['bytes']
        assert hashlib.sha256(data).hexdigest() == meta['sha256'], name
    spec = json.loads(z.read('v4/delta_manifest.json'))
    for name, digest in spec['members'].items():
        assert hashlib.sha256((project/name).read_bytes()).hexdigest() == digest, name
    assert 'Ran 70 tests' in z.read('v4/real_callbacks.log').decode()
    assert 'Ran 13 tests' in z.read('v4/source_callbacks.log').decode()
    assert z.read('v4/real_callbacks.log').decode().strip().endswith('OK')
    assert z.read('v4/source_callbacks.log').decode().strip().endswith('OK')
    dds_names = [n for n in z.namelist() if n.startswith('v4/dds_') and n.endswith('/result.json')]
    assert len(dds_names) == 1
    dds = json.loads(z.read(dds_names[0]))
    assert dds['passed'] and not dds['errors']
    assert dds['stages'][-1]['planner_requests'] == 2
    loss = json.loads(z.read('v4/native_loss_regression.json'))
    assert loss['passed'] and len(loss['runs']) == 3
    for run in loss['runs']:
        assert run['max_callback_gap_ms'] < 500
        assert run['qos'][0]['reliability'] == 2 and run['qos'][0]['depth'] == 1
        assert run['identity']['sha256'] == spec['members']['nav2_adapter/wildos_nav2_adapter/native_vehicle_control.py']
    summaries = {}
    for version, trial in [('v3', 'trial_20261021_084227'), ('v4', 'trial_20261021_085212')]:
        prefix = version+'/full/'+trial+'/'
        frozen = json.loads(z.read(version+'/frozen_source_manifest.json'))['members']
        installed = json.loads(z.read(prefix+'sessions/session_'+trial.removeprefix('trial_')+'/verified_sources.json'))
        runtime = json.loads(z.read(prefix+'runtime_identity.json'))
        installed_libraries = {name: digest for name, digest in installed.items() if name not in frozen}
        assert installed_libraries == {v['path']: v['sha256'] for v in runtime['libraries'].values()}
        assert all(frozen[name] == digest for name, digest in installed.items() if name in frozen)
        config = json.loads(z.read(prefix+'config.json'))
        assert {k: v['sha256'] for k,v in runtime['libraries'].items()} == config['library_sha256']
        result = json.loads(z.read(prefix+'result.json'))
        summary = json.loads(z.read(prefix+'field_summary.json'))
        assert result['closed'] and not result['errors']
        assert result['goal_result']['state'] == 'timeout'
        assert not result['complete_navigation_passed']
        assert abs(summary['valid_s']-120.) < 1e-6
        assert not summary['diagnostic_errors'] and summary['actuator_bad'] == 0
        assert summary['source_rejections'] == {'null': summary['native_samples']}
        assert summary['stop']['samples'] and all(v['speed'] < .03 for v in summary['stop']['samples'])
        post = json.loads(z.read(version+'/post_field.json'))
        assert not post['processes'] and not post['display148_lock']
        summaries[version] = {k: summary[k] for k in
            ('stationary_fraction', 'net_m', 'source_callback_ms', 'successful_reuse', 'dispatch_requests')}
    report = dict(passed=True, archive_members=len(manifest['members']),
        frozen_source_members=len(spec['members']), related_real_ros_tests=83,
        native_loss_runs=3, field_summaries=summaries,
        complete_navigation_passed=False, committed=False,
        scope='Evidence integrity and explicit component/short-field assertions; not full navigation acceptance.')
(root/'local_verification.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
