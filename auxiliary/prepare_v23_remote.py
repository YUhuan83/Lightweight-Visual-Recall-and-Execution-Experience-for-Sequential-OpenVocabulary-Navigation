import hashlib,json,shutil,zipfile
from pathlib import Path
root=Path('/home/ly/execution_continuity_review_20261022')
source=root/'v22_odometry_ingress';target=root/'v23_arc_heading'
assert source.is_dir() and not target.exists()
with zipfile.ZipFile('/tmp/v23_delta.zip') as package:
    base=json.loads(package.read('base_hashes.json'));names=list(base)
    assert set(package.namelist())==set(names+['base_hashes.json'])
    for name in names:assert hashlib.sha256((source/'candidate'/name).read_bytes()).hexdigest()==base[name],name
    target.mkdir();shutil.copytree(source/'candidate',target/'candidate')
    for name in names:(target/'candidate'/name).write_bytes(package.read(name))
full=target/'full';full.mkdir()
for item in (source/'full').iterdir():
    if item.is_file() and (item.suffix=='.py' or item.name in ('config.json','frozen_source_manifest.json')):shutil.copy2(item,full/item.name)
(full/'candidate').symlink_to('../candidate')
for name in ('launch_trial.py','run_full.sh','build.sh','real_ros_review.py'):shutil.copy2(source/name,target/name)
for path in [target/name for name in ('launch_trial.py','run_full.sh','build.sh','real_ros_review.py')]+list(full.glob('*.py'))+[full/'config.json']:
    path.write_text(path.read_text().replace(str(source),str(target)))
path=full/'frozen_source_manifest.json';manifest=json.loads(path.read_text())
for name in names:manifest['members'][name]=hashlib.sha256((target/'candidate'/name).read_bytes()).hexdigest()
path.write_text(json.dumps(manifest,indent=2))
receipt=dict(source=str(source),target=str(target),base='frozen_v22',changed_members={name:manifest['members'][name] for name in names},member_count=len(manifest['members']))
(target/'candidate_delta_v23.json').write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt,indent=2))
