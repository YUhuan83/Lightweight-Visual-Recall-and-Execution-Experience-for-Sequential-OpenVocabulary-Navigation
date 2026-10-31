import hashlib,json,shutil,zipfile
from pathlib import Path
root=Path('/home/ly/execution_continuity_review_20261022')
source=root/'v26_body_arrival';target=root/'v27_fresh_bearing'
assert not target.exists()
with zipfile.ZipFile('/tmp/fresh_bearing_delta.zip') as z:
    expected=json.loads(z.read('base_hashes.json'))
    names=[n for n in z.namelist() if n!='base_hashes.json']
    for name,sha in expected.items():assert hashlib.sha256((source/'candidate'/name).read_bytes()).hexdigest()==sha,name
    target.mkdir();shutil.copytree(source/'candidate',target/'candidate')
    for name in names:
        p=target/'candidate'/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(z.read(name))
full=target/'full';full.mkdir()
for p in (source/'full').iterdir():
    if p.is_file() and (p.suffix=='.py' or p.name in ('config.json','frozen_source_manifest.json')):shutil.copy2(p,full/p.name)
(full/'candidate').symlink_to('../candidate')
for name in ('launch_trial.py','run_full.sh','real_ros_review.py'):shutil.copy2(source/name,target/name)
for p in [target/n for n in ('launch_trial.py','run_full.sh')]+list(full.glob('*.py'))+[full/'config.json']:
    text=p.read_text().replace(str(source),str(target))
    # Motion/arrival packages stay exactly the v26 build; only perception is overlaid.
    text=text.replace(str(target)+'/install/',str(source)+'/install/')
    p.write_text(text)
visual=str(target/'candidate/wildos_port/repro_check/src/nebula2-wildos/visual_navigation')
config=json.loads((full/'config.json').read_text())
config['python_paths'].insert(0,visual)
(full/'config.json').write_text(json.dumps(config,indent=2))
p=target/'run_full.sh';text=p.read_text().replace('export PYTHONPATH=',f'export PYTHONPATH={visual}:',1);p.write_text(text)
manifest=json.loads((full/'frozen_source_manifest.json').read_text())
for name in names:manifest['members'][name]=hashlib.sha256((target/'candidate'/name).read_bytes()).hexdigest()
(full/'frozen_source_manifest.json').write_text(json.dumps(manifest,indent=2))
receipt=dict(base=str(source),target=str(target),changed={n:manifest['members'][n] for n in names},
             motion_build=str(source/'install'),visual_path=visual)
(target/'receipt.json').write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt,indent=2))
