"""Validate sealed evidence identity; NOT navigation success or a repair test."""
import hashlib,json,zipfile
from pathlib import Path
root=Path(__file__).parent
identity=json.loads((root/'smac_full_field_identity.json').read_text())
archive=root/'smac_full_field_evidence_20261018.zip'
assert archive.stat().st_size==identity['bytes']
assert hashlib.sha256(archive.read_bytes()).hexdigest()==identity['sha256']
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None
    index=json.loads(z.read('evidence_manifest.json'))
    assert len(index['files'])==identity['payloads']
    assert not any(n.lower().endswith('.mp4') for n in z.namelist())
    for name,record in index['files'].items():
        data=z.read(name)
        assert len(data)==record['bytes'] and hashlib.sha256(data).hexdigest()==record['sha256'],name
    dest=root/'closed_evidence'
    for name in z.namelist():
        target=(dest/name).resolve()
        assert target.is_relative_to(dest.resolve())
    z.extractall(dest)
session=dest/'session'
review=json.loads((session/'full_query_review.json').read_text())
result=json.loads((dest/'driver/result.json').read_text())
stop=json.loads((session/'goal_001/stop.json').read_text())
running=json.loads((session/'running_identity.json').read_text())
recovery=json.loads((session/'positive_source_recovery.json').read_text())
assert result['goal']['state']=='timeout' and not result['passed']
assert result['component_closed'] and result['session_result']['errors']==[]
assert stop['physics']['speed']<.05 and stop['physics']['bodyCollisionNodeCount']==467
assert all(w['collisionNodeCount']==32 for w in stop['physics']['wheels'])
assert running['libraries']['libwildos_ackermann_smac.so']['sha256']=='fc149c802333ebbea9504e175ae92050821a86a2debb31aaed276fbf1d711a33'
assert recovery['all_resolved'] and recovery['recovered'][0]['retired_generation_not_acknowledged']
assert abs(sum(review['stopped_by_source_reason'].values())-review['stopped_sec'])<1e-6
assert abs(sum(review['stopped_by_sent_control'].values())-review['stopped_sec'])<1e-6
assert review['source_deadline_violations']==[]
assert review['watchdog_samples']==review['missing_receipts']==0
local_manifest=json.loads((root.parents[1]/'deploy/20261018_smoothing_boundary_candidate/official_wildos_beamng_bundle.manifest.json').read_text())
for path in (dest/'candidate').rglob('*.py'):
    name=str(path.relative_to(dest/'candidate')).replace('\\','/')
    assert hashlib.sha256(path.read_bytes()).hexdigest()==local_manifest['members'][name]
report=dict(evidence_verified=True,navigation_success=False,archive_sha256=identity['sha256'],payloads=identity['payloads'],
            verified=['archive_and_members','actual_repaired_library','physical_collision_and_stop',
                      'stopped_time_conservation','retired_inflight_receipt_not_acknowledged','candidate_python_snapshots','no_mp4'])
(root/'verification.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
