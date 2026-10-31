import hashlib,json,shutil,zipfile
from pathlib import Path
root=Path('/home/ly/execution_continuity_review_20261022')
source=root/'v24_continuous_exploration_r2';target=root/'v25_fast_shift'
assert not target.exists()
with zipfile.ZipFile('/tmp/fast_shift_delta.zip') as z:
    base=json.loads(z.read('base_hashes.json'))
    names=[n for n in z.namelist() if n!='base_hashes.json']
    assert len(names)==2
    for name,sha in base.items():assert hashlib.sha256((source/'candidate'/name).read_bytes()).hexdigest()==sha,name
    target.mkdir();shutil.copytree(source/'candidate',target/'candidate')
    for name in names:(target/'candidate'/name).write_bytes(z.read(name))
full=target/'full';full.mkdir()
for p in (source/'full').iterdir():
    if p.is_file() and (p.suffix=='.py' or p.name in ('config.json','frozen_source_manifest.json')):shutil.copy2(p,full/p.name)
(full/'candidate').symlink_to('../candidate')
for name in ('launch_trial.py','run_full.sh','build.sh','real_ros_review.py'):shutil.copy2(source/name,target/name)
for p in [target/n for n in ('launch_trial.py','run_full.sh','build.sh','real_ros_review.py')]+list(full.glob('*.py'))+[full/'config.json']:
    p.write_text(p.read_text().replace(str(source),str(target)))
p=target/'real_ros_review.py';p.write_text(p.read_text().replace('modules = (',"modules = ('test_physical_shift_profile_ros', ",1))
p=full/'frozen_source_manifest.json';manifest=json.loads(p.read_text())
for name in names:manifest['members'][name]=hashlib.sha256((target/'candidate'/name).read_bytes()).hexdigest()
p.write_text(json.dumps(manifest,indent=2))
receipt=dict(source=str(source),target=str(target),changed_members={n:manifest['members'][n] for n in names},member_count=len(manifest['members']),scope='Only physical launch direction debounce 2s->0.3s; default controller 2s, stopped-speed<=0.15m/s, freshness/ACK/physics/planning remain unchanged.')
(target/'fast_shift_delta_receipt.json').write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt,indent=2))
