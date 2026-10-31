from pathlib import Path
import hashlib,json,zipfile
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent
assert json.loads((OUT/'evidence_review.json').read_text())['evidence_consistent']
paths=[p for p in OUT.rglob('*') if p.is_file() and '__pycache__' not in p.parts and
       'server' not in p.relative_to(OUT).parts and p.name not in ('physical_evidence_handoff.zip','archive_manifest.json')]
paths += [ROOT/'docs/PHYSICAL_EVIDENCE_CONTRACT_REPAIR_20261019.md']
members={p.relative_to(ROOT).as_posix():{'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'bytes':p.stat().st_size} for p in sorted(paths)}
archive=OUT/'physical_evidence_handoff.zip'
with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as package:
 for p in sorted(paths):package.write(p,p.relative_to(ROOT).as_posix())
 package.writestr('payload_manifest.json',json.dumps(members,indent=2)+'\n')
with zipfile.ZipFile(archive) as package:
 assert package.testzip() is None and set(package.namelist())==set(members)|{'payload_manifest.json'}
 for name,identity in members.items():
  payload=package.read(name)
  assert len(payload)==identity['bytes'] and hashlib.sha256(payload).hexdigest()==identity['sha256'],name
report=dict(archive=str(archive),payloads=len(members),bytes=archive.stat().st_size,
            sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),crc_and_members_verified=True,
            vehicle_acceptance=False)
(OUT/'archive_manifest.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf8')
print(json.dumps(report,indent=2))
