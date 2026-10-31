#!/usr/bin/env bash
set -eo pipefail
TASK=/home/ly/town_memory_validation_20261014/dds_transport_candidate
PARENT=/home/ly/town_memory_validation_20261014/tracking_contract_candidate
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source "$PARENT/install/setup.bash"
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
python3 - <<'PY'
from pathlib import Path
import hashlib,json,zipfile
task=Path('/home/ly/town_memory_validation_20261014/dds_transport_candidate')
parent=task.parent/'tracking_contract_candidate'
archive=task/'official_wildos_beamng_bundle.zip'
expected='06bb47afd4d32df40afd85fef7aa0f11c97e89efff0c9dcf1e120916da4b64d0'
assert hashlib.sha256(archive.read_bytes()).hexdigest()==expected
manifest=json.loads(archive.with_suffix('.manifest.json').read_text());assert manifest['sha256']==expected
old=json.loads((parent/'candidate.manifest.json').read_text())
allowed={'nav2_adapter/setup.py','nav2_adapter/config/fastdds_local_udp.xml',
    'nav2_adapter/wildos_nav2_adapter/dds_profile.py','nav2_adapter/test/test_dds_profile.py'}
changed=[name for name,digest in manifest['members'].items() if old['members'].get(name)!=digest]
assert all(name.startswith('tools/') or name in allowed for name in changed),changed
assert set(old['members'])<=set(manifest['members'])
root=(task/'bundle').resolve();root.mkdir(exist_ok=False)
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None and set(z.namelist())==set(manifest['members'])
    for name,digest in manifest['members'].items():
        path=(root/name).resolve();assert path.is_relative_to(root)
        data=z.read(name);assert hashlib.sha256(data).hexdigest()==digest
        path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(data)
(task/'changed_sources.json').write_text(json.dumps({'changed':changed,'base':old['sha256'],
    'new':expected,'scope':'DDS environment/preflight/tools; algorithms identical'},indent=2))
print('Verified frozen source; core algorithm source unchanged',expected)
PY
cd "$TASK"
colcon build --base-paths bundle/nav2_adapter --merge-install --packages-select wildos_nav2_adapter --event-handlers console_direct+
source install/setup.bash
python3 - <<'PY'
from pathlib import Path
import hashlib,json
task=Path('/home/ly/town_memory_validation_20261014/dds_transport_candidate')
parent=task.parent/'tracking_contract_candidate'
identity=json.loads((parent/'installation_identity.json').read_text())
report={'passed':True,'scope':'Frozen Python overlay; physical validation pending',
    'base':str(parent/'install'),'base_source_sha256':'46ff51f2355a15fb4f92c79299b774d532dd72432380420e72dae7613978be8e',
    'source_sha256':'06bb47afd4d32df40afd85fef7aa0f11c97e89efff0c9dcf1e120916da4b64d0','files':{}}
source=task/'bundle/nav2_adapter/wildos_nav2_adapter'
installed=task/'install/lib/python3.10/site-packages/wildos_nav2_adapter'
for path in sorted(source.iterdir()):
    if not path.is_file() or path.suffix not in ('.py','.lua'):continue
    assert path.read_bytes()==(installed/path.name).read_bytes(),path.name
    report['files'][path.name]=hashlib.sha256(path.read_bytes()).hexdigest()
for path in sorted((task/'bundle/nav2_adapter/config').iterdir()):
    if path.suffix not in ('.yaml','.xml','.rviz'):continue
    assert path.read_bytes()==(task/'install/share/wildos_nav2_adapter/config'/path.name).read_bytes(),path.name
    report['files']['config/'+path.name]=hashlib.sha256(path.read_bytes()).hexdigest()
for name in ('libwildos_ackermann_rpp.so','libwildos_ackermann_smac.so'):
    digest=hashlib.sha256((parent/'install/lib'/name).read_bytes()).hexdigest()
    assert digest==identity['files'][name];report['files'][name]=digest
(task/'installation_identity.json').write_text(json.dumps(report,indent=2))
print('Python overlay and unchanged actual C++ libraries verified')
PY
/home/ly/beamng_ros2_0394_ws/.venv/bin/python -m unittest discover -s bundle/nav2_adapter/test -p test_dds_profile.py
/home/ly/beamng_ros2_0394_ws/.venv/bin/python -m unittest discover -s bundle/tools -p test_physical_trial_policy.py
