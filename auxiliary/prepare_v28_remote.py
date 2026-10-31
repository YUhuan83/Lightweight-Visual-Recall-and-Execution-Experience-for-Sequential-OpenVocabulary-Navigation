import hashlib,json,shutil,zipfile
from pathlib import Path
root=Path('/home/ly/execution_continuity_review_20261022')
source=root/'v27_fresh_bearing';target=root/'v28_intent_handoff'
assert not target.exists()
with zipfile.ZipFile('/tmp/intent_handoff_delta.zip') as z:
    expected=json.loads(z.read('base_hashes.json'))
    names=[n for n in z.namelist() if n!='base_hashes.json']
    for n,h in expected.items():assert hashlib.sha256((source/'candidate'/n).read_bytes()).hexdigest()==h,n
    target.mkdir();shutil.copytree(source/'candidate',target/'candidate')
    for n in names:(target/'candidate'/n).write_bytes(z.read(n))
full=target/'full';full.mkdir()
for p in (source/'full').iterdir():
    if p.is_file() and (p.suffix=='.py' or p.name in ('config.json','frozen_source_manifest.json')):shutil.copy2(p,full/p.name)
(full/'candidate').symlink_to('../candidate')
for n in ('launch_trial.py','run_full.sh','real_ros_review.py'):shutil.copy2(source/n,target/n)
for p in [target/n for n in ('launch_trial.py','run_full.sh')]+list(full.glob('*.py'))+[full/'config.json']:
    p.write_text(p.read_text().replace(str(source),str(target)))
config=json.loads((full/'config.json').read_text());config['setups'].append(str(target/'install/local_setup.bash'))
(full/'config.json').write_text(json.dumps(config,indent=2))
p=target/'run_full.sh';p.write_text(p.read_text().replace('export PYTHONPATH=',f'source {target}/install/local_setup.bash\nexport PYTHONPATH=',1))
manifest=json.loads((full/'frozen_source_manifest.json').read_text())
for n in names:manifest['members'][n]=hashlib.sha256((target/'candidate'/n).read_bytes()).hexdigest()
(full/'frozen_source_manifest.json').write_text(json.dumps(manifest,indent=2))
build=(root/'v26_body_arrival/build.sh').read_text().replace(str(root/'v26_body_arrival'),str(target))
(target/'build.sh').write_text(build)
receipt=dict(base=str(source),target=str(target),changed={n:manifest['members'][n] for n in names})
(target/'receipt.json').write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt))
