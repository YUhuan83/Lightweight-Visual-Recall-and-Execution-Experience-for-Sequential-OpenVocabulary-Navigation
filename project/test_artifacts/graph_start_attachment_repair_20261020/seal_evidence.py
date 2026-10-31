"""Seal this two-file repair, real C++ tests and closed field evidence."""
from pathlib import Path
import hashlib
import json
import zipfile

root=Path(__file__).resolve().parent;out=root/'sealed';out.mkdir(exist_ok=False)
files={};references=[]
def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        while data:=f.read(4*1024*1024):h.update(data)
    return h.hexdigest()
def add(path,name=None):
    if path.is_file():files[name or str(path.relative_to(root))]=path
for path in root.iterdir():
    if path.is_file() and path.suffix in ('.py','.sh','.json','.log','.xml','.cpp'):add(path)
for path in (root/'harness').iterdir():add(path)
frozen=json.loads((root/'frozen_source_manifest.json').read_text())
for name,digest in frozen['members'].items():
    p=root/'candidate'/name;assert sha(p)==digest,name;add(p)
parent=Path('/home/ly/exploration_progress_repair_20261020/full/candidate')
for name in json.loads((root/'delta_manifest.json').read_text()):add(parent/name,'before/'+name)
for path in (root/'full').iterdir():
    if path.is_file():add(path)
for trial in sorted((root/'full').glob('trial_*')):
    assert json.loads((trial/'result.json').read_text())['closed'],str(trial)
    for p in trial.rglob('*'):
        if not p.is_file():continue
        if p.suffix=='.db3' or p.name in ('native_timing.jsonl','native_udp.jsonl'):
            references.append(dict(path=str(p),size=p.stat().st_size,sha256=sha(p)));continue
        if p.suffix in ('.json','.jsonl','.log','.png','.cdr','.yaml'):add(p)
lib=root/'install/graphnav_mapper/lib/libgraphnav_mapper_component.so'
references.append(dict(path=str(lib),size=lib.stat().st_size,sha256=sha(lib)))
manifest=dict(scope=__doc__,files={n:dict(size=p.stat().st_size,sha256=sha(p)) for n,p in files.items()},
              server_references=references,complete_navigation_passed=False)
(out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
archive=out/'graph_start_evidence.zip'
with zipfile.ZipFile(archive,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
    for name,path in files.items():z.write(path,name)
    z.write(out/'manifest.json','evidence_manifest.json')
report=dict(archive=str(archive),size=archive.stat().st_size,sha256=sha(archive),payloads=len(files),references=len(references))
(out/'archive.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
