"""Supplement the immutable core archive with closed source/shadow diagnostics."""
import hashlib,json,time,zipfile
from pathlib import Path
out=Path('/home/ly/execution_wait_audit_20261018/startup_receipt')
root=Path('/home/ly/planning_phase_physical_20261018')
driver=root/'field_yaw_left90_search_20261018_151916'
d=json.loads((driver/'result.json').read_text());s=Path(d['session'])
assert (s/'session_result.json').is_file() and (s/'goal_001/result.json').is_file()
files={}
for p in out.iterdir():
    if p.is_file() and p.suffix in ('.py','.sh','.log','.json','.jsonl') and p.name!='closed_archive.json':
        files['diagnostics/'+p.name]=p
for name in ('controller_chain_review.json','controller_chain_review.log','closed_handoff_review.json','closed_handoff_review.log','review_controller_chain.py','closed_handoff_review.py'):
    files['field_review/'+name]=root/name
manifest={n:dict(bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for n,p in files.items()}
archive=out/'search_diagnostic_supplement.zip';assert not archive.exists()
with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
    for n,p in files.items():z.write(p,n)
    z.writestr('supplement_manifest.json',json.dumps(manifest,indent=2)+'\n')
report=dict(archive=str(archive),sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),bytes=archive.stat().st_size,payload_members=len(manifest),session=str(s),scope=__doc__)
(out/'search_supplement_identity.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
