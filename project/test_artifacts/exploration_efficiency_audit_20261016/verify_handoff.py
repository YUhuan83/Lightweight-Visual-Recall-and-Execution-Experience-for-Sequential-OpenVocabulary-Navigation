from pathlib import Path
import hashlib
import json
import shutil
import zipfile

root=Path(__file__).resolve().parents[2]
out=root/'test_artifacts/recovery_handoff_repair_20261016'
archive=out/'evidence.zip'
meta=json.loads((out/'evidence_meta.json').read_text())
assert hashlib.sha256(archive.read_bytes()).hexdigest()==meta['sha256']
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None
    manifest=json.loads(z.read('MANIFEST.json'))
    assert len(manifest)==meta['members']
    for name,sha in manifest.items():
        data=z.read(name)
        assert hashlib.sha256(data).hexdigest()==sha,name
        target=out/'server'/name
        assert target.resolve().is_relative_to(out.resolve())
        target.parent.mkdir(parents=True,exist_ok=True)
        target.write_bytes(data)
for name in ('dispatch_gap_before_correct_fixture.log','dispatch_gap_after.log',
             'quarantine_escape_behavior_before.log','quarantine_escape_after.log',
             'handoff_global.log','frontier_alternatives.py','replay_frontiers.sh',
             'validate_recovery_handoff.sh'):
    shutil.copy2(Path(__file__).parent/name,out/name)
for name in ('report.json','report.md','callback_report.json'):
    shutil.copy2(root/'test_artifacts/offline_navigation'/name,out/name)
report=json.loads((out/'report.json').read_text(encoding='utf8'))
callbacks=json.loads((out/'callback_report.json').read_text(encoding='utf8'))
result=dict(evidence=meta,zip_crc_ok=True,members_sha_ok=True,
    offline={s:sum(r['status']==s for r in report['checks']) for s in ('PASS','FAIL','WARN')},
    callbacks_tests=callbacks['tests'],callbacks_passed=callbacks['passed'],
    candidate='415c0bef49adf99f0fd7c64de75d7210973715090c6002fe8742064b92f7dde3',
    deployed=False,vehicle_tested=False,committed=False)
assert result['offline']['FAIL']==0 and callbacks['passed']
(out/'summary.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
