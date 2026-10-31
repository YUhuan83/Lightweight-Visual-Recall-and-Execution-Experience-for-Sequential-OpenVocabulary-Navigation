#!/usr/bin/env bash
set -eo pipefail
TASK=/home/ly/town_memory_validation_20261014/curvature_contract_candidate1
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/town_memory_validation_20261014/tracking_contract_candidate/install/setup.bash
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
cd "$TASK"
source install/setup.bash
colcon test --merge-install --packages-select wildos_ackermann_smac --event-handlers console_direct+
colcon test-result --verbose
python3 - <<'PY'
from pathlib import Path
import hashlib,json
task=Path('/home/ly/town_memory_validation_20261014/curvature_contract_candidate1')
files={}
source=task/'bundle/nav2_adapter/wildos_nav2_adapter'
installed=task/'install/lib/python3.10/site-packages/wildos_nav2_adapter'
for path in source.iterdir():
    if path.is_file() and path.suffix in ('.py','.lua'):
        assert path.read_bytes()==(installed/path.name).read_bytes(),path.name
        files[path.name]=hashlib.sha256(path.read_bytes()).hexdigest()
for path in (task/'bundle/nav2_adapter/config').iterdir():
    if path.suffix in ('.yaml','.xml','.rviz'):
        assert path.read_bytes()==(task/'install/share/wildos_nav2_adapter/config'/path.name).read_bytes(),path.name
        files['config/'+path.name]=hashlib.sha256(path.read_bytes()).hexdigest()
for path in (task/'bundle/ackermann_smac/include/wildos_ackermann_smac').iterdir():
    assert path.read_bytes()==(task/'install/include/wildos_ackermann_smac'/path.name).read_bytes(),path.name
files['libwildos_ackermann_smac.so']=hashlib.sha256((task/'install/lib/libwildos_ackermann_smac.so').read_bytes()).hexdigest()
parent=task.parent/'tracking_contract_candidate'
old=json.loads((parent/'installation_identity.json').read_text())
files['libwildos_ackermann_rpp.so']=hashlib.sha256((parent/'install/lib/libwildos_ackermann_rpp.so').read_bytes()).hexdigest()
assert files['libwildos_ackermann_rpp.so']==old['files']['libwildos_ackermann_rpp.so']
(task/'installation_identity.json').write_text(json.dumps({'scope':'Installed candidate; physical validation pending','files':files},indent=2))
PY
python3 -m unittest discover -s bundle/nav2_adapter/test -p test_graph_route.py
python3 -m unittest discover -s bundle/nav2_adapter/test -p test_dds_profile.py
python3 -m unittest discover -s bundle/tools -p test_physical_trial_policy.py
