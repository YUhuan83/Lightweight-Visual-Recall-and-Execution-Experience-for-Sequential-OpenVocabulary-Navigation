"""Create a separate frozen r2; leave v23 and initial v24 intact."""
import hashlib,json,shutil,zipfile
from pathlib import Path
root=Path('/home/ly/execution_continuity_review_20261022')
source=root/'v24_continuous_exploration'
target=root/'v24_continuous_exploration_r2'
assert source.is_dir() and not target.exists()
with zipfile.ZipFile('/tmp/continuous_r2_delta.zip') as z:
    base=json.loads(z.read('base_hashes.json'))
    names=[n for n in z.namelist() if n!='base_hashes.json']
    assert len(names)==6 and all(n.startswith(('nav2_adapter/','tools/')) for n in names)
    for name,sha in base.items():
        assert hashlib.sha256((source/'candidate'/name).read_bytes()).hexdigest()==sha,name
    target.mkdir()
    shutil.copytree(source/'candidate',target/'candidate')
    for name in names:
        p=target/'candidate'/name
        p.parent.mkdir(parents=True,exist_ok=True)
        p.write_bytes(z.read(name))
full=target/'full'
full.mkdir()
for p in (source/'full').iterdir():
    if p.is_file() and (p.suffix=='.py' or p.name in ('config.json','frozen_source_manifest.json')):
        shutil.copy2(p,full/p.name)
(full/'candidate').symlink_to('../candidate')
for name in ('launch_trial.py','run_full.sh','build.sh','real_ros_review.py'):
    shutil.copy2(source/name,target/name)
for p in [target/n for n in ('launch_trial.py','run_full.sh','build.sh','real_ros_review.py')]+list(full.glob('*.py'))+[full/'config.json']:
    p.write_text(p.read_text().replace(str(source),str(target)))
p=target/'real_ros_review.py'
s=p.read_text()
assert s.count('modules = (')==1
s=s.replace('modules = (',"modules = ('test_continuous_process_roles', ",1)
p.write_text(s)
p=target/'build.sh'
s=p.read_text()
needle='export PYTHONPATH=$PWD/candidate/nav2_adapter/test:$PYTHONPATH'
assert s.count(needle)==1
p.write_text(s.replace(needle,'export PYTHONPATH=$PWD/candidate/tools:$PWD/candidate/nav2_adapter/test:$PYTHONPATH'))
p=full/'frozen_source_manifest.json'
manifest=json.loads(p.read_text())
for name in names:
    manifest['members'][name]=hashlib.sha256((target/'candidate'/name).read_bytes()).hexdigest()
p.write_text(json.dumps(manifest,indent=2))
receipt=dict(source=str(source),target=str(target),execution_mode='continuous_exploration_v1',
    changed_members={name:manifest['members'][name] for name in names},member_count=len(manifest['members']),
    scope='Entry recognition and nominal planner radius proposal contract; old source retained. No physical/controller/watchdog/dwell configuration changes.')
(target/'continuous_r2_delta_receipt.json').write_text(json.dumps(receipt,indent=2))
print(json.dumps(receipt,indent=2))
