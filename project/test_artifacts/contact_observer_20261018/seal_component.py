"""Freeze closed diagnostic evidence; raw sensor SQLite remains on server."""
from pathlib import Path
import hashlib
import json
import sys
import zipfile

root=Path(__file__).resolve().parent
run=Path(sys.argv[1])
result=json.loads((run/'result.json').read_text())
review=json.loads((run/'review.json').read_text())
assert result['passed'] and not result['errors'] and review['component_passed']
assert result['motion_goals']==0 and review['positive_drive_commands']==0
assert review['unload']['unloaded'] is True
assert review['complete_navigation_passed'] is False
for name,want in result['diagnostic_files'].items():
    assert hashlib.sha256((root/name).read_bytes()).hexdigest()==want,name
payloads={str(Path('run')/name):run/name for name in (
    'result.json','review.json','contact_result.json','running_identity.json',
    'owned_processes.json','prestart.log','scene.json','scene.samples.jsonl',
    'sensor.json','stack.log','engine.log','sensor.log','recorder.log',
    'contact_probe.log','runtime_identity.log','traffic.json','native_timing.jsonl',
    'interfaces/metadata.yaml')}
for name in result['diagnostic_files']:payloads['diagnostic/'+name]=root/name
for name in ('lua_fixture_tests.json','lua_fixture_tests.log',
             'lua_fixture_before_ownership.json','lua_fixture_before_ownership.log',
             'review_component.py','seal_component.py'):
    payloads['diagnostic/'+name]=root/name
manifest={name:dict(bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest())
          for name,p in payloads.items()}
db=next((run/'interfaces').glob('*.db3'))
external=dict(path=str(db),bytes=db.stat().st_size,sha256=hashlib.sha256(db.read_bytes()).hexdigest(),
              local_raw_database_downloaded=False)
assert external['sha256']==review['database_sha256']
archive=run/'contact_component_evidence.zip'
with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as zf:
    for name,p in payloads.items():zf.write(p,name)
    zf.writestr('manifest.json',json.dumps(manifest,indent=2)+'\n')
    zf.writestr('external_raw_interface_manifest.json',json.dumps(external,indent=2)+'\n')
with zipfile.ZipFile(archive) as zf:
    assert zf.testzip() is None
    for name,want in manifest.items():
        data=zf.read(name)
        assert len(data)==want['bytes'] and hashlib.sha256(data).hexdigest()==want['sha256']
identity=dict(archive=str(archive),bytes=archive.stat().st_size,
              sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),payloads=len(manifest),external=external)
(run/'evidence_identity.json').write_text(json.dumps(identity,indent=2)+'\n')
print(json.dumps(identity,indent=2))
