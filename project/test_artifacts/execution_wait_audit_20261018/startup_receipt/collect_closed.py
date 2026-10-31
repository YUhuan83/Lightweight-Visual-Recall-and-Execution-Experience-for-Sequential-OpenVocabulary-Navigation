"""Seal closed component and isolated DDS evidence; no active databases."""
import hashlib,json,zipfile
from pathlib import Path
root=Path('/home/ly/planning_phase_physical_20261018')
out=Path('/home/ly/execution_wait_audit_20261018/startup_receipt')
driver=root/'field_yaw_left90_lifecycle_20261018_151059'
assert (driver/'result.json').is_file()
result=json.loads((driver/'result.json').read_text())
session=Path(result['session']);assert (session/'session_result.json').is_file()
assert result['passed'] is True,result
selected={}
for label,s in [('failed_093826',root/'field_results/session_20261018_093826'),('passed_151103',session)]:
    assert (s/'session_result.json').is_file(),s
    for name in ('session_result.json','server_preflight.json','verified_sources.json','stop.json',
                 'console_identity.json','owned_processes.json','stack.log','preflight.log',
                 'execution_observer/messages.jsonl','execution_observer/ready.json'):
        path=s/name
        if path.is_file():selected[label+'/'+name]=path
for trial in [driver,root/'field_yaw_left90_lifecycle_20261018_093821']:
    for path in trial.iterdir():
        if path.is_file() and path.suffix in ('.json','.log'):
            selected[trial.name+'/'+path.name]=path
for path in out.iterdir():
    if path.is_file() and path.suffix in ('.json','.jsonl','.log','.py','.sh'):
        selected['diagnostics/'+path.name]=path
for name in ('freeze.json','post_run_verification.json','manifest.json'):
    path=root/name
    if path.exists():selected['frozen/'+name]=path
manifest={name:dict(bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest(),server_path=str(p)) for name,p in selected.items()}
archive=out/'closed_startup_component.zip';assert not archive.exists()
with zipfile.ZipFile(archive,'x',compression=zipfile.ZIP_DEFLATED) as z:
    for name,p in selected.items():z.write(p,name)
    z.writestr('evidence_manifest.json',json.dumps(manifest,indent=2)+'\n')
report=dict(archive=str(archive),sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),bytes=archive.stat().st_size,
            payload_members=len(manifest),driver_passed=result['passed'],session_errors=result['session_result']['errors'],scope=__doc__)
(out/'closed_archive.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
