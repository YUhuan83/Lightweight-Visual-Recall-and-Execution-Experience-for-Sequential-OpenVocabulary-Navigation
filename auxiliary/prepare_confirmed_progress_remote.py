import hashlib,json,shutil,zipfile
from pathlib import Path
base=Path('/home/ly/execution_continuity_review_20261022')
source=base/'v28_intent_handoff';target=base/'v31_confirmed_progress'
assert not target.exists()
with zipfile.ZipFile(base/'confirmed_progress_delta.zip') as archive:
    expected=json.loads(archive.read('base_hashes.json'))
    for name,digest in expected.items():
        assert hashlib.sha256((source/'candidate'/name).read_bytes()).hexdigest()==digest,name
    names=[n for n in archive.namelist() if n!='base_hashes.json']
    target.mkdir();shutil.copytree(source/'candidate',target/'candidate',ignore=shutil.ignore_patterns('__pycache__'))
    for name in names:
        p=target/'candidate'/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(archive.read(name))
full=target/'full';full.mkdir()
for p in (source/'full').iterdir():
    if p.is_file() and (p.suffix=='.py' or p.name in ('config.json','frozen_source_manifest.json')):
        shutil.copy2(p,full/p.name)
(full/'candidate').symlink_to('../candidate')
for name in ('launch_trial.py','run_full.sh','real_ros_review.py'):
    shutil.copy2(source/name,target/name)
for p in [target/'launch_trial.py',target/'run_full.sh']+list(full.glob('*.py'))+[full/'config.json']:
    p.write_text(p.read_text().replace(str(source),str(target)))
manifest=json.loads((full/'frozen_source_manifest.json').read_text())
for name in names:manifest['members'][name]=hashlib.sha256((target/'candidate'/name).read_bytes()).hexdigest()
(full/'frozen_source_manifest.json').write_text(json.dumps(manifest,indent=2))
review=target/'real_ros_review.py'
text=review.read_text();assert 'modules = (' in text
review.write_text(text.replace('modules = (',"modules = ('test_confirmed_progress_ros', ",1))
build='''#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v9/run_full.sh)
cd /home/ly/execution_continuity_review_20261022/v31_confirmed_progress
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
colcon --log-base build_log build --base-paths candidate/nav2_adapter candidate/official_adapter --packages-select wildos_nav2_adapter beamng_wildos_adapter --build-base build --install-base install --parallel-workers 2 >build.log 2>&1
source install/local_setup.bash
export ROS_DOMAIN_ID=205 ROS_LOCALHOST_ONLY=1
export PYTHONPATH=$PWD/candidate/wildos_port/repro_check/src/nebula2-wildos/visual_navigation:$PWD/candidate/tools:$PWD/candidate/nav2_adapter/test:$PYTHONPATH
python real_ros_review.py >real_ros_review.log 2>&1
'''
(target/'build.sh').write_text(build)
receipt=dict(base=str(source),target=str(target),changed={n:manifest['members'][n] for n in names})
(target/'receipt.json').write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt))
