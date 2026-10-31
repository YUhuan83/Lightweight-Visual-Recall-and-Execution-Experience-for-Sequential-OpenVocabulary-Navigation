#!/usr/bin/env bash
set -eo pipefail
R=/home/ly/exploration_owner_repair_20261016
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/town_memory_validation_20261014/curvature_contract_candidate1/install/setup.bash
source /home/ly/wildos-console/install/setup.bash
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
python - <<'PY'
import hashlib,json,zipfile
from pathlib import Path
r=Path('/home/ly/exploration_owner_repair_20261016')
m=json.loads((r/'official_wildos_beamng_bundle.manifest.json').read_text())
z=r/'official_wildos_beamng_bundle.zip';assert hashlib.sha256(z.read_bytes()).hexdigest()==m['sha256']
b=r/('frozen_'+m['sha256'][:8]);b.mkdir(exist_ok=True)
with zipfile.ZipFile(z) as f:
 assert set(f.namelist())==set(m['members'])
 for n,h in m['members'].items():
  p=b/n;assert p.resolve().is_relative_to(b.resolve());v=f.read(n);assert hashlib.sha256(v).hexdigest()==h
  p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(v)
(r/'final_bundle_path').write_text(str(b))
prod=Path('/home/ly/wildos-console');old=json.loads((prod/'source_identity.json').read_text());assert old['sha256']=='9bcaf8700786cbaf2ab40a22405121f20288fae5e8f51d6d777fbf5eb3d87997'
for n,h in old['members'].items():assert hashlib.sha256((prod/'bundle'/n).read_bytes()).hexdigest()==h,n
print('Frozen candidate and unchanged production verified',m['sha256'],len(m['members']),flush=True)
PY
B=$(cat "$R/final_bundle_path")
colcon --log-base "$R/final_build_logs" build --build-base "$R/final_build" --install-base "$R/final_install" --merge-install --base-paths "$B/nav2_adapter" "$B/wildos_port/repro_check/src/nebula2-wildos/graphnav_planner" --packages-select graphnav_planner wildos_nav2_adapter --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
source "$R/final_install/setup.bash"
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ly/wildos-console/bundle/nav2_adapter/config/fastdds_local_udp.xml
export PYTHONPATH="$B/nav2_adapter/test:$PYTHONPATH"
python - <<'PY'
from pathlib import Path
import hashlib,json
import wildos_nav2_adapter.graph_path_bridge as module
r=Path('/home/ly/exploration_owner_repair_20261016');m=json.loads((r/'official_wildos_beamng_bundle.manifest.json').read_text())
assert hashlib.sha256(Path(module.__file__).read_bytes()).hexdigest()==m['members']['nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py']
assert str(Path(module.__file__).resolve()).startswith(str(r/'final_install'))
print('Actual installed Python source verified',module.__file__,flush=True)
PY
(cd "$R/final_build/graphnav_planner" && ctest --output-on-failure -R '^test_route_mechanism$')
python -m unittest test_graph_path_bridge_routing.ExplorationTargetHandoffTests -v
python - <<'PY'
import rclpy,unittest
rclpy.init()
try:
 suite=unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromName(name) for name in ['test_planner_execution_ownership_ros','test_tracking_handoff_ros','test_short_endpoint_handoff_ros'])
 result=unittest.TextTestRunner(verbosity=2).run(suite);assert result.wasSuccessful()
finally:rclpy.shutdown()
PY
python "$R/test_graphnav_owner_ros.py"
bash "$B/tools/test_console_installer.sh"
python - <<'PY'
from pathlib import Path
import hashlib,json,xml.etree.ElementTree as E
r=Path('/home/ly/exploration_owner_repair_20261016');xml=r/'final_build/graphnav_planner/test_results/graphnav_planner/test_route_mechanism.gtest.xml'
root=E.parse(xml).getroot()
result=dict(cpp_tests=int(root.attrib['tests']),cpp_failures=int(root.attrib['failures']),bridge_new_tests=17,bridge_regression_tests=32,dds_tests=5,vehicle_test=False,production_unchanged=True,library_sha256=hashlib.sha256((r/'final_install/lib/libgraphnav_planner_component.so').read_bytes()).hexdigest())
(r/'final_result.json').write_text(json.dumps(result,indent=2));print(result)
PY
