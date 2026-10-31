"""Archive the preserved candidates, counterexamples and final bounded validations."""
from pathlib import Path
import hashlib,json,zipfile
out=Path(__file__).resolve().parent;prior=out.parent/'goal_release_repair_20261019'
inventory=json.loads((out/'server/post_inventory.json').read_text(encoding='utf-8-sig'))
assert inventory['processes']==[]
archive=out/'goal_release_handoff_evidence.zip'
excluded={archive.name,'archive_manifest.json'}
files=[p for directory in (prior,out) for p in directory.rglob('*')
       if p.is_file() and '__pycache__' not in p.parts and p.name not in excluded]
payloads={p.relative_to(out.parent).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
    for p in files:z.write(p,p.relative_to(out.parent).as_posix())
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None and set(z.namelist())==set(payloads)
    for name,want in payloads.items():assert hashlib.sha256(z.read(name)).hexdigest()==want,name
manifest=dict(archive=archive.name,bytes=archive.stat().st_size,sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
    payloads=payloads,vehicle_acceptance=False,production_deployment=False)
(out/'archive_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf8')
print(json.dumps({k:v for k,v in manifest.items() if k!='payloads'}|dict(payload_count=len(payloads)),indent=2))
