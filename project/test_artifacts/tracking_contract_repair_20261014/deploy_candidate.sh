#!/usr/bin/env bash
set -eo pipefail
TASK=/home/ly/town_memory_validation_20261014/tracking_contract_candidate
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
export CMAKE_BUILD_PARALLEL_LEVEL=4 ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1
python3 - <<'PY'
from pathlib import Path
import hashlib,json,zipfile
task=Path('/home/ly/town_memory_validation_20261014/tracking_contract_candidate')
archive=task/'candidate.zip'
expected='46ff51f2355a15fb4f92c79299b774d532dd72432380420e72dae7613978be8e'
assert hashlib.sha256(archive.read_bytes()).hexdigest()==expected
manifest=json.loads((task/'candidate.manifest.json').read_text())
assert manifest['sha256']==expected
root=(task/'bundle').resolve();root.mkdir(exist_ok=False)
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None
    assert set(z.namelist())==set(manifest['members'])
    for name,digest in manifest['members'].items():
        target=(root/name).resolve()
        assert target.is_relative_to(root) and not target.is_symlink()
        data=z.read(name)
        assert hashlib.sha256(data).hexdigest()==digest
        target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
print('verified',len(manifest['members']),'source files',expected)
PY
cd "$TASK"
colcon build --base-paths bundle --merge-install --packages-select \
 graphnav_msgs object_search_msgs graphnav_mapper graphnav_planner visual_navigation \
 beamng_wildos_adapter wildos_nav2_adapter wildos_ackermann_rpp wildos_ackermann_smac \
 --event-handlers console_direct+
source install/setup.bash
python3 - <<'PY'
from pathlib import Path
import hashlib,json
task=Path('/home/ly/town_memory_validation_20261014/tracking_contract_candidate')
report={'passed':True,'scope':'installation identity, not vehicle acceptance','files':{}}
for rel in ['graph_path_bridge.py','tracking_preview.py','runtime_contract.py',
            'continuous_path.py','navigation_transactions.py','memory_selection.py',
            'cmd_vel_to_control.py','native_vehicle_control.py','execution_feedback.py',
            'vehicle_control_lease.lua']:
    source=task/'bundle/nav2_adapter/wildos_nav2_adapter'/rel
    installed=task/'install/lib/python3.10/site-packages/wildos_nav2_adapter'/rel
    a=hashlib.sha256(source.read_bytes()).hexdigest();b=hashlib.sha256(installed.read_bytes()).hexdigest()
    assert a==b,rel
    report['files'][rel]=b
source=task/'bundle/nav2_adapter/config/nav2_ackermann_control.yaml'
installed=task/'install/share/wildos_nav2_adapter/config/nav2_ackermann_control.yaml'
assert source.read_bytes()==installed.read_bytes()
report['files']['nav2_ackermann_control.yaml']=hashlib.sha256(installed.read_bytes()).hexdigest()
for name in ['libwildos_ackermann_rpp.so','libwildos_ackermann_smac.so']:
    report['files'][name]=hashlib.sha256((task/'install/lib'/name).read_bytes()).hexdigest()
(task/'installation_identity.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
PY
