"""Verify the sealed coordinate audit and extract into a new local evidence directory."""
from pathlib import Path,PurePosixPath
import hashlib
import json
import zipfile

root=Path(__file__).resolve().parent;archive=root/'frame_evidence.zip'
identity=json.loads((root/'evidence_identity.json').read_text())
assert archive.stat().st_size==identity['bytes']
assert hashlib.sha256(archive.read_bytes()).hexdigest()==identity['sha256']
target=root/'closed_evidence';assert not target.exists()
with zipfile.ZipFile(archive) as zf:
    assert zf.testzip() is None
    manifest=json.loads(zf.read('manifest.json'));assert len(manifest)==identity['payloads']
    assert set(zf.namelist())==set(manifest)|{'manifest.json','external_raw_interfaces.json'}
    for name,want in manifest.items():
        data=zf.read(name)
        assert len(data)==want['bytes'] and hashlib.sha256(data).hexdigest()==want['sha256']
    for name in zf.namelist():
        item=PurePosixPath(name)
        assert not item.is_absolute() and '..' not in item.parts and '\\' not in name
        path=target.joinpath(*item.parts)
        assert path.resolve().is_relative_to(target.resolve())
        path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(zf.read(name))
result=dict(archive_sha256=identity['sha256'],verified_payloads=len(manifest),crc_passed=True,
    raw_database_downloaded=False,extracted=str(target),complete_navigation_passed=False)
(root/'local_archive_verification.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
