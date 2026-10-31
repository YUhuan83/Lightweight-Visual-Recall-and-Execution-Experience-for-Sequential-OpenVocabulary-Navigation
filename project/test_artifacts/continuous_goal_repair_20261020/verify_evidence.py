"""Verify both closed server archives, then copy the tested fixture and compact results locally."""
from pathlib import Path, PurePosixPath
import hashlib
import json
import zipfile
import xml.etree.ElementTree as ET

root=Path(__file__).resolve().parent
project=root.parent.parent
archives=json.loads((root/'evidence_archives.json').read_text())
summary={}
for item in archives:
    archive=root/PurePosixPath(item['path']).name
    assert archive.stat().st_size==item['bytes']
    assert hashlib.sha256(archive.read_bytes()).hexdigest()==item['sha256']
    with zipfile.ZipFile(archive) as z:
        assert z.testzip() is None
        manifest=json.loads(z.read('EVIDENCE_MANIFEST.json'))
        assert set(z.namelist())=={'EVIDENCE_MANIFEST.json',*manifest['members']}
        for name,digest in manifest['members'].items():
            pure=PurePosixPath(name)
            assert not pure.is_absolute() and '..' not in pure.parts and ':' not in name
            assert hashlib.sha256(z.read(name)).hexdigest()==digest,name
        summary[archive.name]=dict(members=len(manifest['members']),sha256=item['sha256'],
                                   raw_bags=manifest['raw_bags_retained_on_server'])
        if archive.name=='continuous_goal_evidence.zip':
            # Copy the exact fixture included in the actual compiled C++ test.
            fixture='ackermann_smac/test/fixtures/reverse_goal_public.txt'
            target=project/fixture
            assert not target.exists() or target.read_bytes()==z.read('candidate/'+fixture)
            target.write_bytes(z.read('candidate/'+fixture))
            for name in ('ackermann_smac/src/planner.cpp','ackermann_smac/test/test_planner.cpp'):
                assert (project/name).read_bytes()==z.read('candidate/'+name),name
            frozen=json.loads(z.read('physical/frozen_source_manifest.json'))
            differences=[n for n,h in frozen['members'].items()
                         if not (project/n).is_file() or hashlib.sha256((project/n).read_bytes()).hexdigest()!=h]
            assert not differences,differences
            summary['frozen_source_members']=len(frozen['members'])
            for name in ('physical_summary.json','after_inventory.json','closed_map_review/result.json',
                         'after_test_planner.xml','after_test_connector.xml','after_test_path_contract.xml',
                         'before_tests.xml','risk_fixture_v1_failed.xml',
                         'physical/short_20261020_164936/execution/result.json',
                         'physical/short_20261020_164936/result.json',
                         'physical/short_20261020_164936/runtime_libraries.json'):
                target=root/'verified'/name
                target.parent.mkdir(parents=True,exist_ok=True)
                target.write_bytes(z.read(name))
            tests={}
            for name in ('test_planner','test_connector','test_path_contract'):
                xml=ET.fromstring(z.read(f'after_{name}.xml'))
                tests[name]={n:int(xml.attrib[n]) for n in ('tests','failures','errors')}
                assert tests[name]['failures']==tests[name]['errors']==0
            summary['actual_cpp_tests']=tests
            summary['physical']=json.loads(z.read('physical_summary.json'))
            assert summary['physical']['passed'] and not summary['physical']['complete_navigation_passed']
(root/'local_verification.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
