"""Seal CPU-only planning evidence; never collect MP4 or active databases."""
import hashlib,json,zipfile
from pathlib import Path
root=Path('/home/ly/vehicle_interface_audit_20261018')
repair=Path('/home/ly/smoothing_boundary_repair_20261018')
assert all((repair/n).is_file() for n in ('planner_tests.log','connector_tests.log','path_contract_tests.log','library_identity.txt'))
files={}
for p in root.rglob('*'):
    if not p.is_file() or any(v in ('build','__pycache__') for v in p.relative_to(root).parts):continue
    if p.suffix in ('.py','.sh','.cpp','.hpp','.txt','.csv','.log','.json','.diff'):
        files['diagnostic/'+p.relative_to(root).as_posix()]=p
for n in ('build.log','planner_tests.log','connector_tests.log','path_contract_tests.log','library_identity.txt','official_wildos_beamng_bundle.manifest.json',
          'candidate/ackermann_smac/src/planner.cpp','candidate/ackermann_smac/test/test_planner.cpp'):
    files['repair/'+n]=repair/n
manifest={n:dict(bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for n,p in files.items()}
archive=root/'planning_root_cause_evidence.zip';assert not archive.exists()
with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
    for n,p in files.items():z.write(p,n)
    z.writestr('evidence_manifest.json',json.dumps(manifest,indent=2)+'\n')
report=dict(archive=str(archive),bytes=archive.stat().st_size,sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),payload_members=len(manifest),scope=__doc__)
(root/'evidence_identity.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
