import hashlib,json,zipfile,subprocess
from pathlib import Path
root=Path('/home/ly/wildos-console');bundle=root/'bundle'
expected='ffcfb67df7c433150b97849b57c14096fa0cbeb4347676d257d61dce3db0fa50'
a=Path('/home/ly/wildos_console_ffcfb67d.zip')
assert hashlib.sha256(a.read_bytes()).hexdigest()==expected
old=json.loads((root/'source_identity.json').read_text());assert old['sha256'].startswith('ff0fa6e5')
with zipfile.ZipFile(a) as z:
 assert z.testzip() is None
 hashes={n:hashlib.sha256(z.read(n)).hexdigest() for n in z.namelist()}
 changed={n for n in set(hashes)|set(old['members']) if hashes.get(n)!=old['members'].get(n)}
 assert changed=={'nav2_adapter/test/test_graph_path_bridge_routing.py','nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py','tools/test_wildos_console.py','tools/wildos_console.py'},changed
 bundle.rename(root/'bundle_ff0fa6e5');bundle.mkdir();z.extractall(bundle)
 assert all(hashlib.sha256((bundle/n).read_bytes()).hexdigest()==h for n,h in hashes.items())
(root/'source_identity.json').rename(root/'source_identity_ff0fa6e5.json')
(root/'source_identity.json').write_text(json.dumps(dict(sha256=expected,members=hashes,changed=sorted(changed)),indent=2))
script=r'''#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/town_memory_validation_20261014/curvature_contract_candidate1/install/setup.bash
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
colcon --log-base /home/ly/wildos-console/build_logs build --build-base /home/ly/wildos-console/build --install-base /home/ly/wildos-console/install --merge-install --base-paths /home/ly/wildos-console/bundle/nav2_adapter --packages-select wildos_nav2_adapter
source /home/ly/wildos-console/install/setup.bash
export ROS_DOMAIN_ID=89
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ly/wildos-console/bundle/nav2_adapter/config/fastdds_local_udp.xml
export PYTHONPATH=/home/ly/wildos-console/bundle/nav2_adapter/test:$PYTHONPATH
/usr/bin/python3 -m unittest test_graph_path_bridge_routing -v
/usr/bin/python3 -m unittest discover -s /home/ly/wildos-console/bundle/tools -p test_wildos_console.py
/usr/bin/python3 -c "from pathlib import Path; Path('/home/ly/wildos-console/collision_gate_passed.json').write_text('{\"passed\":true}')"
'''
p=root/'collision_build_test.sh';p.write_text(script)
assert not (root/'collision_gate_passed.json').exists()
with (root/'collision_build_test.log').open('xb') as log:
 child=subprocess.Popen(['bash',str(p)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
(root/'collision_build_test.pid').write_text(str(child.pid));print(child.pid)
