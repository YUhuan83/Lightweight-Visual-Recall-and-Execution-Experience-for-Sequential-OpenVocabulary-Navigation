from pathlib import Path, PurePosixPath
import hashlib,json,zipfile,collections
root=Path(__file__).parent
identity=json.loads((root/'field_archive_identity.json').read_text(encoding='utf8'))
archive=root/'full_vehicle_field_evidence.zip'
assert hashlib.sha256(archive.read_bytes()).hexdigest()==identity['sha256']
target=root/'closed_evidence';target.mkdir(exist_ok=True)
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None
    manifest=json.loads(z.read('field_evidence_manifest.json'))
    assert len(z.namelist())==len(manifest['files'])+1==identity['members']
    for name,info in manifest['files'].items():
        p=PurePosixPath(name)
        assert not p.is_absolute() and '..' not in p.parts and ':' not in name
        data=z.read(name)
        assert len(data)==info['bytes'] and hashlib.sha256(data).hexdigest()==info['sha256'],name
        if p.suffix=='.mp4':
            video=Path('captures/20261018_motion_handoff/rviz_live.mp4')
            assert hashlib.sha256(video.read_bytes()).hexdigest()==info['sha256']
        else:
            out=target/name;out.parent.mkdir(exist_ok=True,parents=True);out.write_bytes(data)
    (target/'field_evidence_manifest.json').write_bytes(z.read('field_evidence_manifest.json'))
report=dict(zip_crc=True,sha256=identity['sha256'],verified_payloads=len(manifest['files']),
    video_verified_separately=True,external_raw_sensor_bag=identity['external_files'])
(root/'local_validation.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf8')
print(json.dumps(report,indent=2))
