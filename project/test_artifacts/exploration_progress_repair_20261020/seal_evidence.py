"""Seal only completed exploration trials and diagnostic snapshots; never active SQLite."""
from pathlib import Path
import hashlib
import json
import zipfile

root=Path(__file__).resolve().parent;full=root/'full';destination=root/'sealed';destination.mkdir(exist_ok=False)
files={};references=[]
def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as source:
        while chunk:=source.read(4*1024*1024):h.update(chunk)
    return h.hexdigest()
def add(path,name=None):
    if path.is_file():files[name or str(path.relative_to(root))]=path
for path in root.iterdir():
    if path.is_file() and path.suffix in ('.py','.sh','.json','.log'):add(path)
for path in full.iterdir():
    if path.is_file() and path.suffix in ('.py','.sh','.json','.log','.txt','.cdr','.cpp'):add(path)
for path in (full/'terrain_start_review').glob('*'):add(path)
for trial_name in ('trial_20261020_174256','trial_20261020_174700','trial_20261020_180816'):
    trial=full/trial_name
    result=json.loads((trial/'result.json').read_text());assert result['closed']
    for path in trial.rglob('*'):
        if not path.is_file():continue
        if path.suffix=='.db3' or path.name in ('native_timing.jsonl','native_udp.jsonl'):
            references.append(dict(path=str(path),size=path.stat().st_size,sha256=digest(path),closed_trial=trial_name));continue
        if path.suffix in ('.json','.jsonl','.log','.png','.cdr','.yaml'):add(path)
manifest=dict(scope=__doc__,files={name:dict(size=p.stat().st_size,sha256=digest(p)) for name,p in files.items()},
              server_references=references,production_changed=False,complete_navigation_passed=False)
(destination/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
archive=destination/'exploration_evidence.zip'
with zipfile.ZipFile(archive,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
    for name,path in files.items():z.write(path,name)
    z.write(destination/'manifest.json','evidence_manifest.json')
report=dict(archive=str(archive),size=archive.stat().st_size,sha256=digest(archive),payloads=len(files),references=len(references))
(destination/'archive.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
