"""Verify the closed contact-component archive and explicit acceptance limits."""
from pathlib import Path
import hashlib
import json
import zipfile

root=Path(__file__).resolve().parent
archive=root/'contact_component_evidence.zip'
digest=hashlib.sha256(archive.read_bytes()).hexdigest()
assert digest=='1c3193eecf13b4d95d78e7e9c36e50668537e6af52550514123420f4c9649a74'
with zipfile.ZipFile(archive) as zf:
    assert zf.testzip() is None
    manifest=json.loads(zf.read('manifest.json'))
    assert len(manifest)==30
    for name,want in manifest.items():
        data=zf.read(name)
        assert len(data)==want['bytes'] and hashlib.sha256(data).hexdigest()==want['sha256'],name
    review=json.loads(zf.read('run/review.json'))
    before=json.loads(zf.read('diagnostic/lua_fixture_before_ownership.json'))
    after=json.loads(zf.read('diagnostic/lua_fixture_tests.json'))
    assert before['failures']==1 and not before['passed'] and after['tests']==11 and after['passed']
    assert review['event_totals']==dict(body=0,wheel=6300,unknown=0)
    assert review['unload']['unloaded'] is True and review['positive_drive_commands']==0
    assert review['maximum_speed']<.05 and review['complete_navigation_passed'] is False
    assert review['measurement_clock_mapping_topic_recorded'] is False
    assert review['cross_stream_clock_mapping_verified'] is False
    assert review['observer_watchdog_frames'][0]['telemetry_ok'] is False
    assert review['observer_watchdog_frames'][0]['telemetry_age_sec'] is None
    assert review['observer_watchdog_frames'][0]['command_age']<.02
    for name in ('run/review.json','run/result.json','run/running_identity.json',
                 'diagnostic/lua_fixture_tests.json','diagnostic/lua_fixture_before_ownership.log',
                 'external_raw_interface_manifest.json'):
        (root/Path(name).name).write_bytes(zf.read(name))
report=dict(archive_sha256=digest,payloads=30,all_hashes_verified=True,
            component_verified=True,real_body_contact_branch_verified=False,
            cross_stream_clock_mapping_verified=False,historical_stall_reproduced=False,
            complete_navigation_passed=False)
(root/'verification.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
