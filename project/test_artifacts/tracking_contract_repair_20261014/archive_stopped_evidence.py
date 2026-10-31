"""Archive closed diagnostic files only; no simulator or process interaction."""
from pathlib import Path
import hashlib,json,zipfile
BASE=Path('/home/ly/town_memory_validation_20261014')
candidate=BASE/'tracking_contract_candidate'
files=[]
audit=BASE/'controller_audit'
files.extend(p for p in audit.iterdir() if p.is_file() and p.suffix in ('.log','.xml','.csv','.gz'))
files.extend(p for p in (audit/'src').rglob('*') if p.is_file())
files.extend(p for p in (audit/'build').rglob('*.xml') if 'test_results' in p.parts)
for folder in ('physical_component_session','physical_cusp_component1','physical_cusp_component2','component_tools'):
    files.extend(p for p in (candidate/folder).rglob('*') if p.is_file() and '__pycache__' not in p.parts)
files.extend(p for p in candidate.iterdir() if p.is_file() and p.suffix in ('.log','.json','.sh'))
files=sorted(set(files))
manifest={p.relative_to(BASE).as_posix():{'bytes':p.stat().st_size,
    'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in files}
archive=BASE/'tracking_contract_stopped_evidence.zip'
assert not archive.exists()
with zipfile.ZipFile(archive,'x',compression=zipfile.ZIP_DEFLATED) as z:
    for p in files:z.write(p,p.relative_to(BASE).as_posix())
    z.writestr('EVIDENCE_MANIFEST.json',json.dumps(manifest,indent=2))
metadata={'file':str(archive),'bytes':archive.stat().st_size,'members':len(files),
    'sha256':hashlib.sha256(archive.read_bytes()).hexdigest()}
(BASE/'tracking_contract_stopped_evidence.json').write_text(json.dumps(metadata,indent=2)+'\n')
print(json.dumps(metadata,indent=2))
