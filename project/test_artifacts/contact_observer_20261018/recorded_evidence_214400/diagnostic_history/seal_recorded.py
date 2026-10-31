"""Seal closed diagnostic histories; leave sensor databases on the server."""
from pathlib import Path
import hashlib
import json
import sys
import zipfile

root=Path(__file__).resolve().parent
run=root/'recorded_20261018_214400'
result=json.loads((run/'result.json').read_text())
review=json.loads((run/'review_recorded.json').read_text())
checks=json.loads((run/'review_consistency.json').read_text())
inventory=json.loads((root/'end_inventory.json').read_text())
assert result['component_closed'] and result['errors']==[] and result['passed'] is False
assert checks['passed'] and not review['complete_navigation_passed']
assert inventory['processes']==[] and not inventory['display148_lock']
payloads={}
history=[];external=[]
for name in ('recorded_20261018_214046','recorded_20261018_214221','recorded_20261018_214400'):
    directory=root/name
    old=json.loads((directory/'result.json').read_text())
    history.append(dict(run=name,parent_result=old['passed'],parent_errors=old['errors'],
        parent_default_motion_count_not_authoritative=(name=='recorded_20261018_214221'),
        scope='Diagnostic readiness/UUID logging failure' if name!=run.name else 'Actual admitted section stalled with body contacts'))
    for path in directory.rglob('*'):
        if not path.is_file() or path.suffix in ('.db3','.zip') or path.name.endswith(('-wal','-shm')):continue
        payloads['runs/'+str(path.relative_to(root))]=path
    for db in (directory/'interfaces').glob('*.db3'):
        external.append(dict(path=str(db),bytes=db.stat().st_size,sha256=hashlib.sha256(db.read_bytes()).hexdigest(),local_raw_downloaded=False))
for name,want in result['diagnostic_files'].items():
    path=root/name
    assert hashlib.sha256(path.read_bytes()).hexdigest()==want,name
    payloads['frozen_diagnostic/'+name]=path
for path in root.iterdir():
    if path.is_file() and (path.suffix in ('.py','.sh','.lua') or path.name.startswith('recorded_ros_input_tests')):
        payloads['diagnostic_history/'+path.name]=path
payloads['end_inventory.json']=root/'end_inventory.json'
payloads['frozen_fixture.json']=root/'recorded_section_fixture.json'
manifest={name:dict(bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for name,p in payloads.items()}
archive=run/'recorded_section_evidence.zip'
with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as zf:
    for name,path in payloads.items():zf.write(path,name)
    zf.writestr('manifest.json',json.dumps(manifest,indent=2)+'\n')
    zf.writestr('external_raw_interfaces.json',json.dumps(external,indent=2)+'\n')
    zf.writestr('history.json',json.dumps(history,indent=2)+'\n')
with zipfile.ZipFile(archive) as zf:
    assert zf.testzip() is None
    for name,want in manifest.items():
        data=zf.read(name)
        assert len(data)==want['bytes'] and hashlib.sha256(data).hexdigest()==want['sha256'],name
identity=dict(archive=str(archive),bytes=archive.stat().st_size,sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
    payloads=len(manifest),databases=external,complete_navigation_passed=False,history=history)
(run/'evidence_identity.json').write_text(json.dumps(identity,indent=2)+'\n')
print(json.dumps(identity,indent=2))
