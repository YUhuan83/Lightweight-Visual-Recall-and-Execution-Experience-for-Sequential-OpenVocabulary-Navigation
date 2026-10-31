"""Freeze the async-state candidate separately; preserve all v4 evidence."""
from pathlib import Path
import hashlib, json, shutil, zipfile
root=Path('/home/ly/execution_continuity_review_20261022')
old,new=root/'v4',root/'v5'
assert not new.exists()
shutil.copytree(old/'candidate',new/'candidate',symlinks=True)
manifest=json.loads((old/'full/frozen_source_manifest.json').read_text())
with zipfile.ZipFile(root/'incoming/serialized_results_delta.zip') as archive:
    assert archive.testzip() is None
    for name in archive.namelist():
        target=new/'candidate'/name
        assert target.resolve().is_relative_to((new/'candidate').resolve())
        target.parent.mkdir(parents=True,exist_ok=True)
        data=archive.read(name);target.write_bytes(data)
        manifest['members'][name]=hashlib.sha256(data).hexdigest()
full=new/'full';full.mkdir()
for path in (old/'full').glob('*.py'):shutil.copy2(path,full/path.name)
(full/'candidate').symlink_to(new/'candidate',target_is_directory=True)
assert 'ROUND_TIMEOUT_SEC=180.0' in (full/'driver_full.py').read_text()
(full/'frozen_source_manifest.json').write_text(json.dumps(manifest,indent=2))
config=json.loads((old/'full/config.json').read_text())
config['setups'].append(str(new/'install/local_setup.bash'))
config['python_paths'].insert(0,str(new/'candidate/wildos_port/repro_check/src/nebula2-wildos'))
config['output_root']=str(full/'sessions')
config['goal_timeout_s']=180
(full/'config.json').write_text(json.dumps(config,indent=2))
shutil.copy2(old/'real_ros_review.py',new/'real_ros_review.py')
shell=(old/'run_full.sh').read_text().replace(str(old),str(new))
shell=shell.replace('source '+str(root/'v3/install/local_setup.bash')+'\n',
    'source '+str(root/'v3/install/local_setup.bash')+'\nsource '+str(old/'install/local_setup.bash')+'\n')
(new/'run_full.sh').write_text(shell)
(new/'build.sh').write_text('''#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v4/run_full.sh)
cd /home/ly/execution_continuity_review_20261022/v5
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
colcon --log-base build_log build --base-paths candidate/nav2_adapter candidate/official_adapter --packages-select wildos_nav2_adapter beamng_wildos_adapter --build-base build --install-base install --parallel-workers 2 >build.log 2>&1
source install/local_setup.bash
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
export ROS_DOMAIN_ID=205 ROS_LOCALHOST_ONLY=1
export PYTHONPATH=$PWD/candidate/nav2_adapter/test:$PYTHONPATH
python real_ros_review.py >real_ros_review.log 2>&1
''')
print(json.dumps({'destination':str(new),'source_members':len(manifest['members'])}))
