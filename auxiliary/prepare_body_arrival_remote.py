import hashlib,json,shutil,zipfile
from pathlib import Path
root=Path('/home/ly/execution_continuity_review_20261022')
source=root/'v25_fast_shift';target=root/'v26_body_arrival'
assert not target.exists(),target
with zipfile.ZipFile('/tmp/body_arrival_delta.zip') as z:
    expected=json.loads(z.read('base_hashes.json'))
    names=[n for n in z.namelist() if n!='base_hashes.json']
    assert len(names)==10
    for name,sha in expected.items():assert hashlib.sha256((source/'candidate'/name).read_bytes()).hexdigest()==sha,name
    target.mkdir();shutil.copytree(source/'candidate',target/'candidate')
    for name in names:
        p=target/'candidate'/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(z.read(name))
full=target/'full';full.mkdir()
for p in (source/'full').iterdir():
    if p.is_file() and (p.suffix=='.py' or p.name in ('config.json','frozen_source_manifest.json')):shutil.copy2(p,full/p.name)
(full/'candidate').symlink_to('../candidate')
for name in ('launch_trial.py','run_full.sh','build.sh','real_ros_review.py'):shutil.copy2(source/name,target/name)
for p in [target/n for n in ('launch_trial.py','run_full.sh','build.sh','real_ros_review.py')]+list(full.glob('*.py'))+[full/'config.json']:
    p.write_text(p.read_text().replace(str(source),str(target)))
p=target/'real_ros_review.py'
s=p.read_text();assert s.count('modules = (')==1
s=s.replace('modules = (',"modules = ('test_body_arrival_ros', 'test_execution_mechanism_ros', 'test_arrival_criteria', 'test_surface_association_ros', 'test_multi_goal_session_ros', ",1)
p.write_text(s)
p=full/'frozen_source_manifest.json';manifest=json.loads(p.read_text())
for name in names:manifest['members'][name]=hashlib.sha256((target/'candidate'/name).read_bytes()).hexdigest()
p.write_text(json.dumps(manifest,indent=2))
receipt=dict(source=str(source),target=str(target),changed_members={n:manifest['members'][n] for n in names},member_count=len(manifest['members']),body_distance_m=2.,arrival_basis='vehicle_body_clearance')
(target/'body_arrival_receipt.json').write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt,indent=2))
