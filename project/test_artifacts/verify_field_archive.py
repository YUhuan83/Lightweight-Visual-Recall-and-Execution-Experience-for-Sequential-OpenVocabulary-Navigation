"""Verify every archived payload, then extract bounded diagnostics only."""
from pathlib import Path
import hashlib,json,sys,zipfile

directory=Path(sys.argv[1]).resolve()
identity=json.loads((directory/'field_archive_identity.json').read_text(encoding='utf8'))
archive=directory/Path(identity['path']).name
def digest(stream):
    h=hashlib.sha256()
    for chunk in iter(lambda:stream.read(4*1024*1024),b''):h.update(chunk)
    return h.hexdigest()
with archive.open('rb') as stream:assert digest(stream)==identity['sha256']
out=directory/'server_field';out.mkdir(exist_ok=True)
with zipfile.ZipFile(archive) as bundle:
    assert bundle.testzip() is None
    metadata=json.loads(bundle.read('field_evidence_manifest.json'))
    assert set(bundle.namelist())==set(metadata['files'])|{'field_evidence_manifest.json'}
    for name,want in metadata['files'].items():
        with bundle.open(name) as stream:assert digest(stream)==want['sha256'],name
        assert bundle.getinfo(name).file_size==want['bytes'],name
        target=(out/name).resolve()
        assert target.is_relative_to(out),name
        if target.suffix not in ('.mp4','.db3') and want['bytes']<30*1024*1024:
            target.parent.mkdir(parents=True,exist_ok=True)
            target.write_bytes(bundle.read(name))
    (out/'field_evidence_manifest.json').write_text(json.dumps(metadata,indent=2)+'\n',encoding='utf8')
    if len(sys.argv)>2:
        video=Path(sys.argv[2])
        with video.open('rb') as stream:video_sha=digest(stream)
        main=[v['sha256'] for n,v in metadata['files'].items() if n.endswith('/rviz_live.mp4')]
        assert video_sha in main,(str(video),video_sha)
result=dict(archive_sha256=identity['sha256'],verified_members=len(metadata['files']),crc_passed=True,
    external_files=metadata['external_files'],scope='Core evidence complete; external raw sensor DB remains on server')
(directory/'field_archive_validation.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
print(json.dumps(result,indent=2))
