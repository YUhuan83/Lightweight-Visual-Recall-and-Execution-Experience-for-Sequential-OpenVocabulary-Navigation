"""Seal both closed coordinate-audit runs and exact diagnostic source versions."""
from pathlib import Path
import hashlib
import json
import zipfile

root=Path(__file__).resolve().parent
main=root/'frame_20261019_084254'
inventory=json.loads((root/'end_inventory.json').read_text())
assert inventory['processes']==[] and not inventory['display148_lock']
consistency=json.loads((main/'frame_review_consistency.json').read_text())
assert consistency['passed'] and consistency['complete_navigation_passed'] is False
payloads={};external=[]
possible=[p for p in root.iterdir() if p.is_file()]
for run_name in ('frame_20261019_083934','frame_20261019_084254'):
    run=root/run_name;result=json.loads((run/'result.json').read_text())
    assert result['component_closed'] and result['errors']==[] and not result['passed']
    for name,want in result['diagnostic_files'].items():
        candidates=[p for p in possible if hashlib.sha256(p.read_bytes()).hexdigest()==want]
        assert candidates,(run_name,name,want)
        payloads['frozen/'+run_name+'/'+name]=candidates[0]
    for p in run.rglob('*'):
        if p.is_file() and p.suffix not in ('.db3','.zip') and not p.name.endswith(('-wal','-shm')):
            payloads['runs/'+str(p.relative_to(root))]=p
    for db in (run/'interfaces').glob('*.db3'):
        external.append(dict(path=str(db),bytes=db.stat().st_size,sha256=hashlib.sha256(db.read_bytes()).hexdigest(),local_raw_downloaded=False))
for p in possible:
    if p.suffix in ('.py','.lua','.sh') or p.name.endswith('_tests.json') or p.name.endswith('_tests.log') or p.name in ('preparation.json','recorded_section_fixture.json','end_inventory.json'):
        payloads['diagnostic/'+p.name]=p
manifest={name:dict(bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for name,p in payloads.items()}
archive=main/'frame_evidence.zip'
with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as zf:
    for name,p in payloads.items():zf.write(p,name)
    zf.writestr('manifest.json',json.dumps(manifest,indent=2)+'\n')
    zf.writestr('external_raw_interfaces.json',json.dumps(external,indent=2)+'\n')
with zipfile.ZipFile(archive) as zf:
    assert zf.testzip() is None
    for name,want in manifest.items():
        data=zf.read(name)
        assert len(data)==want['bytes'] and hashlib.sha256(data).hexdigest()==want['sha256']
identity=dict(archive=str(archive),bytes=archive.stat().st_size,sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),payloads=len(manifest),external_raw=external,complete_navigation_passed=False)
(main/'evidence_identity.json').write_text(json.dumps(identity,indent=2)+'\n')
print(json.dumps(identity,indent=2))
