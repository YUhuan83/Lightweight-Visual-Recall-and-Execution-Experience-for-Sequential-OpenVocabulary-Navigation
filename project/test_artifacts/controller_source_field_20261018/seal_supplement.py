"""Seal the post-archive corrections and isolated regression candidates separately."""
from pathlib import Path
import hashlib,json,zipfile
OUT=Path(__file__).parent;ROOT=OUT.parents[1]
files=[]
for folder in ('controller_source_contract_20261018','controller_deadline_race_20261018',
               'controller_deadline_clock_20261018'):
    directory=OUT.parent/folder
    for p in directory.rglob('*'):
        if p.is_file() and '__pycache__' not in p.parts and p.suffix in ('.py','.sh','.json','.log','.zip'):
            files.append(p)
for name in ('closed_trace.json','exact_stop_attribution.py','exact_stop_attribution.log',
             'verified_handoff_review.json','verify_handoff_observations.py','test_closed_evidence.py',
             'evidence_tests.log','local_validation.json','freeze.json'):
    files.append(OUT/name)
files.append(ROOT/'docs/CONTROLLER_SOURCE_CONTRACT_20261018.md')
manifest={p.relative_to(ROOT).as_posix():dict(bytes=p.stat().st_size,
    sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in sorted(set(files))}
archive=OUT/'controller_contract_supplement.zip'
with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
    for name in manifest:z.write(ROOT/name,name)
    z.writestr('supplement_manifest.json',json.dumps(manifest,indent=2)+'\n')
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None
    for name,info in manifest.items():
        data=z.read(name)
        assert len(data)==info['bytes'] and hashlib.sha256(data).hexdigest()==info['sha256'],name
identity=dict(path=str(archive),bytes=archive.stat().st_size,
    sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),verified_payloads=len(manifest),zip_crc=True)
(OUT/'supplement_identity.json').write_text(json.dumps(identity,indent=2)+'\n',encoding='utf8')
print(json.dumps(identity,indent=2))
