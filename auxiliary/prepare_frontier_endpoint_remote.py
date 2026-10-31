import hashlib,json,shutil,zipfile
from pathlib import Path
base=Path('/home/ly/execution_continuity_review_20261022');source=base/'v54_confirmed_graph_handoff';target=base/'v55_frontier_endpoint'
assert not target.exists()
with zipfile.ZipFile(base/'frontier_endpoint_delta.zip') as z:
    expected=json.loads(z.read('base_hashes.json'))
    for n,h in expected.items():assert hashlib.sha256((source/'candidate'/n).read_bytes()).hexdigest()==h,n
    target.mkdir();shutil.copytree(source/'candidate',target/'candidate',ignore=shutil.ignore_patterns('__pycache__'))
    names=[n for n in z.namelist() if n!='base_hashes.json']
    for n in names:
        p=target/'candidate'/n;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(z.read(n))
full=target/'full';full.mkdir()
for p in (source/'full').iterdir():
    if p.is_file() and (p.suffix=='.py' or p.name in ('config.json','frozen_source_manifest.json')):shutil.copy2(p,full/p.name)
(full/'candidate').symlink_to('../candidate')
for n in ('launch_trial.py','run_full.sh','real_ros_review.py'):shutil.copy2(source/n,target/n)
for p in [target/'launch_trial.py',target/'run_full.sh']+list(full.glob('*.py'))+[full/'config.json']:
    p.write_text(p.read_text().replace(str(source),str(target)))

manifest=json.loads((full/'frozen_source_manifest.json').read_text())
for n in names:manifest['members'][n]=hashlib.sha256((target/'candidate'/n).read_bytes()).hexdigest()
(full/'frozen_source_manifest.json').write_text(json.dumps(manifest,indent=2))
(target/'receipt.json').write_text(json.dumps(dict(base=str(source),target=str(target),changed={n:manifest['members'][n] for n in names}),indent=2))
print(target)
