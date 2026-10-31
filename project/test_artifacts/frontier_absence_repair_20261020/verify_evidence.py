"""Verify closed source/build/DDS/entry evidence; no ROS or vehicle required."""
from pathlib import Path, PurePosixPath
import hashlib
import json
import zipfile
import xml.etree.ElementTree as ET

root=Path(__file__).resolve().parent;project=root.parent.parent
identity=json.loads((root/'sealed/identity.json').read_text());archive=root/'sealed/evidence.zip'
assert archive.stat().st_size==identity['bytes']
assert hashlib.sha256(archive.read_bytes()).hexdigest()==identity['sha256']
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None
    manifest=json.loads(z.read('EVIDENCE_MANIFEST.json'))
    assert set(z.namelist())=={'EVIDENCE_MANIFEST.json',*manifest['members']}
    assert len(manifest['members'])==identity['members']
    for name,digest in manifest['members'].items():
        p=PurePosixPath(name)
        assert not p.is_absolute() and '..' not in p.parts and ':' not in name
        assert hashlib.sha256(z.read(name)).hexdigest()==digest,name
    frozen=json.loads(z.read('frozen_source_manifest.json'))
    for name,digest in frozen['members'].items():
        assert hashlib.sha256((project/name).read_bytes()).hexdigest()==digest,name
    for name,digest in json.loads(z.read('before_manifest.json')).items():
        assert hashlib.sha256((root/'before'/name).read_bytes()).hexdigest()==digest,name
    before=ET.fromstring(z.read('before_tests.xml'));after=ET.fromstring(z.read('after_tests.xml'))
    assert int(before.attrib['tests'])==42 and int(before.attrib['failures'])==2
    assert int(after.attrib['tests'])==43 and int(after.attrib['failures'])==int(after.attrib['errors'])==0
    dds=json.loads(z.read('dds_20261020_171520/result.json'))
    entry=json.loads(z.read('entry_20261020_171927/result.json'))
    callbacks=json.loads(z.read('candidate/test_artifacts/frontier_absence_repair_20261020/real_ros.json'))
    assert dds['passed'] and not dds['errors'] and len(dds['stages'])==6
    assert dds['accepted_actions']==dds['cancel_requests']==1 and dds['old_action_terminal_status']==5
    assert entry['passed'] and not entry['errors']
    assert entry['actual_parameters']['missing_frontier_timeout_seconds']==10.
    assert callbacks['passed'] and callbacks['tests']==123
    assert not json.loads(z.read('after_inventory.json'))['processes']
    for name in ('before_tests.xml','after_tests.xml','after_inventory.json',
                 'dds_20261020_171235/result.json','dds_20261020_171520/result.json',
                 'entry_20261020_171927/result.json'):
        p=root/'verified'/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(z.read(name))
    summary=dict(passed=True,archive=identity,frozen_source_members=len(frozen['members']),
                 actual_cpp=43,actual_ros_callbacks=123,dds_stages=6,
                 installed_source_checks=len(entry['verified_install_sources']),
                 installed_default_missing_window_seconds=10.,complete_navigation_passed=False)
(root/'local_verification.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
