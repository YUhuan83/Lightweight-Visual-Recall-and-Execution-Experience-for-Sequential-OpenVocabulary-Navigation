"""Local SHA/CRC verification of the sealed terrain investigation."""
from pathlib import Path,PurePosixPath
import hashlib
import json
import zipfile

root=Path(__file__).resolve().parent;archive=root/'terrain_contract_evidence.zip'
identity=json.loads((root/'evidence_identity.json').read_text())
assert archive.stat().st_size==identity['bytes']
assert hashlib.sha256(archive.read_bytes()).hexdigest()==identity['sha256']
target=root/'closed_evidence';assert not target.exists()
with zipfile.ZipFile(archive) as zf:
    assert zf.testzip() is None
    manifest=json.loads(zf.read('manifest.json'));assert len(manifest)==identity['payloads']
    assert set(zf.namelist())==set(manifest)|{'manifest.json'}
    for name,want in manifest.items():
        data=zf.read(name)
        assert len(data)==want['bytes'] and hashlib.sha256(data).hexdigest()==want['sha256'],name
    for name in zf.namelist():
        item=PurePosixPath(name)
        assert not item.is_absolute() and '..' not in item.parts and '\\' not in name
        path=target.joinpath(*item.parts)
        assert path.resolve().is_relative_to(target.resolve())
        path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(zf.read(name))
report=dict(archive_sha256=identity['sha256'],payloads_verified=len(manifest),crc_passed=True,
    extracted=str(target),raw_database_downloaded=False,complete_navigation_passed=False)
(root/'verification.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
