#!/usr/bin/env bash
set -eo pipefail
TASK=/home/ly/town_memory_validation_20261014/curvature_contract_candidate1
PARENT=/home/ly/town_memory_validation_20261014/tracking_contract_candidate
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source "$PARENT/install/setup.bash"
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
python3 - <<'PY'
from pathlib import Path
import hashlib,json,zipfile
task=Path('/home/ly/town_memory_validation_20261014/curvature_contract_candidate1')
old=Path('/home/ly/town_memory_validation_20261014/dds_transport_candidate/official_wildos_beamng_bundle.manifest.json')
expected='71f9ed53b5e0262c400017f7751efef5f5dcac929ade0809343d7d1f43047660'
archive=task/'candidate.zip';assert hashlib.sha256(archive.read_bytes()).hexdigest()==expected
manifest=json.loads((task/'candidate.manifest.json').read_text());assert manifest['sha256']==expected
previous=json.loads(old.read_text());changed=[n for n,h in manifest['members'].items() if previous['members'].get(n)!=h]
allowed={'nav2_adapter/wildos_nav2_adapter/graph_route.py','nav2_adapter/test/test_graph_route.py',
    'nav2_adapter/test/fixtures/town_smoothing_cusp_20261014.json'}
assert all(n.startswith(('ackermann_smac/','tools/')) or n in allowed for n in changed),changed
assert set(previous['members'])<=set(manifest['members'])
root=(task/'bundle').resolve();root.mkdir(exist_ok=False)
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None and set(z.namelist())==set(manifest['members'])
    for name,digest in manifest['members'].items():
        path=(root/name).resolve();assert path.is_relative_to(root)
        data=z.read(name);assert hashlib.sha256(data).hexdigest()==digest
        path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(data)
(task/'source_identity.json').write_text(json.dumps({'sha256':expected,'base':previous['sha256'],'changed':changed},indent=2))
print('Verified',len(manifest['members']),'frozen files; allowed differences',changed)
PY
cd "$TASK"
colcon build --base-paths bundle/ackermann_smac bundle/nav2_adapter --merge-install --packages-select wildos_ackermann_smac wildos_nav2_adapter --event-handlers console_direct+
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
