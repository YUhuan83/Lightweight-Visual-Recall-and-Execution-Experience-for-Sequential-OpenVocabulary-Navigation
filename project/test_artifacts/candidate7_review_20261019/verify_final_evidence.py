from pathlib import Path, PurePosixPath
import hashlib
import json
import tarfile

OUT = Path(__file__).resolve().parent
archive = OUT / 'evidence_v2.tgz'
want = (OUT / 'evidence_v2.sha256').read_text().split()[0]
assert hashlib.sha256(archive.read_bytes()).hexdigest() == want
dest = OUT / 'server_evidence_v2'
dest.mkdir(exist_ok=False)
with tarfile.open(archive, 'r:gz') as package:
    members = package.getmembers()
    assert len({m.name for m in members}) == len(members)
    for member in members:
        name = PurePosixPath(member.name)
        assert member.isfile() and not name.is_absolute() and '..' not in name.parts
        target = dest.joinpath(*name.parts)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(package.extractfile(member).read())
manifest = json.loads((dest / 'manifest_v2.json').read_text())
for name, expected in manifest.items():
    data = dest.joinpath(*PurePosixPath(name).parts).read_bytes()
    assert len(data) == expected['bytes']
    assert hashlib.sha256(data).hexdigest() == expected['sha256'], name
source = json.loads((dest / 'curved_source_manifest.json').read_text())
assert source == json.loads((OUT / 'source_manifest.json').read_text())
replay = json.loads((dest / 'mapper_review_20261019_214636/result.json').read_text())
assert replay['completed'] and not replay['errors'] and len(replay['snapshots']) == 22
assert replay['terrain_source']['sha256'] == source['official_adapter/beamng_wildos_adapter/terrain.py']
assert json.loads((dest / 'end_inventory.json').read_text())['processes'] == []
for name, count in (('curved_official_tests.log', 51), ('curved_pointcloud_tests.log', 23),
                    ('curved_cloud_filter_tests.log', 1)):
    log = (dest / name).read_text()
    assert 'Ran %d test' % count in log and '\nOK\n' in log
report = dict(passed=True, sha256=want, bytes=archive.stat().st_size,
              members=len(members), payloads=len(manifest),
              complete_navigation_passed=False)
(OUT / 'evidence_v2_review.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf8')
print(json.dumps(report, indent=2))
