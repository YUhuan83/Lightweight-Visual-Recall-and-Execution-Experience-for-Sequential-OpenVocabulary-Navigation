#!/usr/bin/env bash
set -eo pipefail
R=/home/ly/exploration_efficiency_audit_20261016
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/town_memory_validation_20261014/curvature_contract_candidate1/install/setup.bash
source /home/ly/wildos-console/install/setup.bash
source /home/ly/exploration_owner_repair_20261016/final_install/setup.bash
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
python - <<'PY'
import hashlib,json,zipfile
from pathlib import Path
r=Path('/home/ly/exploration_efficiency_audit_20261016');m=json.loads((r/'official_wildos_beamng_bundle.manifest.json').read_text());a=r/'official_wildos_beamng_bundle.zip';assert hashlib.sha256(a.read_bytes()).hexdigest()==m['sha256']
b=r/'candidate';b.mkdir(exist_ok=False)
with zipfile.ZipFile(a) as z:
 assert set(z.namelist())==set(m['members'])
 for n,h in m['members'].items():
  p=b/n;assert p.resolve().is_relative_to(b.resolve());v=z.read(n);assert hashlib.sha256(v).hexdigest()==h;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(v)
old=json.loads(Path('/home/ly/wildos-console/source_identity.json').read_text());diff={n for n,h in m['members'].items() if old['members'].get(n)!=h}
assert diff=={'nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py','nav2_adapter/wildos_nav2_adapter/execution_recovery.py','nav2_adapter/test/test_graph_path_bridge_routing.py'},diff
(r/'candidate_delta.json').write_text(json.dumps(dict(source=m['sha256'],base=old['sha256'],changed=sorted(diff)),indent=2))
PY
colcon --log-base "$R/build_logs" build --build-base "$R/build" --install-base "$R/install" --merge-install --base-paths "$R/candidate/nav2_adapter" --packages-select wildos_nav2_adapter
source "$R/install/setup.bash"
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ly/wildos-console/bundle/nav2_adapter/config/fastdds_local_udp.xml
export PYTHONPATH="$R/candidate/nav2_adapter/test:$PYTHONPATH"
python -m unittest test_graph_path_bridge_routing.ExplorationTargetHandoffTests -v
python - <<'PY'
import unittest,rclpy
rclpy.init()
try:
 s=unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromName(n) for n in ['test_execution_mechanism_ros','test_planner_execution_ownership_ros','test_tracking_handoff_ros','test_short_endpoint_handoff_ros'])
 result=unittest.TextTestRunner(verbosity=2).run(s);assert result.wasSuccessful()
finally:rclpy.shutdown()
PY
python /home/ly/exploration_owner_repair_20261016/test_graphnav_owner_ros.py
