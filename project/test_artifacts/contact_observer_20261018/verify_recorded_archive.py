"""Verify SHA/CRC and extract the sealed recorded-section audit to a new directory."""
from pathlib import Path,PurePosixPath
import hashlib
import json
import zipfile

root=Path(__file__).resolve().parent
archive=root/'recorded_section_evidence.zip'
identity=json.loads((root/'evidence_identity.json').read_text())
assert archive.stat().st_size==identity['bytes']
assert hashlib.sha256(archive.read_bytes()).hexdigest()==identity['sha256']
target=root/'recorded_evidence_214400'
assert not target.exists()
with zipfile.ZipFile(archive) as zf:
    assert zf.testzip() is None
    manifest=json.loads(zf.read('manifest.json'))
    assert len(manifest)==identity['payloads']
    for name,want in manifest.items():
        data=zf.read(name)
        assert len(data)==want['bytes'] and hashlib.sha256(data).hexdigest()==want['sha256'],name
    assert set(zf.namelist())==set(manifest)|{'manifest.json','history.json','external_raw_interfaces.json'}
    for name in zf.namelist():
        parts=PurePosixPath(name)
        assert not parts.is_absolute() and '..' not in parts.parts and '\\' not in name
        path=target.joinpath(*parts.parts)
        assert path.resolve().is_relative_to(target.resolve())
        path.parent.mkdir(parents=True,exist_ok=True)
        path.write_bytes(zf.read(name))
report=dict(archive_sha256=identity['sha256'],verified_payloads=len(manifest),
    crc_passed=True,raw_database_downloaded=False,extracted=str(target),
    complete_navigation_passed=False)
(root/'recorded_archive_verification.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
