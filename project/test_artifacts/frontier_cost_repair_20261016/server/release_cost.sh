#!/usr/bin/env bash
set -eo pipefail
R=/home/ly/frontier_cost_repair_20261016
source /home/ly/exploration_efficiency_audit_20261016/env.sh
source "$R/install/setup.bash"
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
python - <<'PY'
from pathlib import Path
import hashlib,json,zipfile
r=Path('/home/ly/frontier_cost_repair_20261016');m=json.loads((r/'release.manifest.json').read_text())
assert m['sha256']=='e6f2811833ea740887cd9bf08e44814699dbff850646aab9ed01701da381cbeb'
assert hashlib.sha256((r/'release.zip').read_bytes()).hexdigest()==m['sha256']
b=r/'release_candidate';b.mkdir(exist_ok=False)
with zipfile.ZipFile(r/'release.zip') as z:
 assert set(z.namelist())==set(m['members'])
 for n,h in m['members'].items():
  data=z.read(n);assert hashlib.sha256(data).hexdigest()==h
  p=b/n;assert p.resolve().is_relative_to(b.resolve());p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(data)
cpp=[n for n in m['members'] if any(n.startswith('wildos_port/repro_check/src/nebula2-wildos/'+p+'/') for p in ('graphnav_msgs','graphnav_planner'))]
for n in cpp:assert hashlib.sha256((r/'candidate'/n).read_bytes()).hexdigest()==m['members'][n],n
(r/'compiled_source_attestation.json').write_text(json.dumps(dict(final=m['sha256'],compiled_sources_identical=True,members=cpp),indent=2))
PY
colcon --log-base "$R/release_build_logs" build --build-base "$R/release_build" --install-base "$R/release_install" --merge-install --base-paths "$R/release_candidate/nav2_adapter" --packages-select wildos_nav2_adapter
source "$R/release_install/setup.bash"
export PYTHONPATH="$R/release_candidate/nav2_adapter/test:/home/ly/exploration_owner_repair_20261016:$PYTHONPATH"
python -m unittest test_exploration_cost_ros.ExplorationCostTests -v
python -m unittest test_graph_path_bridge_routing.ExplorationTargetHandoffTests -v
python - <<'PY'
import rclpy,unittest
rclpy.init()
try:
 names=['test_planner_execution_ownership_ros','test_tracking_handoff_ros','test_short_endpoint_handoff_ros']+['test_navigation_transactions_ros.NavigationTransactionTests.'+n for n in ('test_escape_waits_for_old_terminal_result_and_settled_stop','test_new_query_revokes_escape_intent_during_cancellation_wait','test_expired_or_moved_escape_intent_falls_back_after_stop')]
 import test_execution_mechanism_ros as m
 names += ['test_execution_mechanism_ros.ExecutionMechanismTests.'+n for n in unittest.defaultTestLoader.getTestCaseNames(m.ExecutionMechanismTests) if any(x in n for x in ('escape','recover'))]
 result=unittest.TextTestRunner(verbosity=2).run(unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromName(n) for n in names));assert result.wasSuccessful()
finally:rclpy.shutdown()
PY
python "$R/test_frontier_choice_dds.py"
bash "$R/release_candidate/tools/test_console_installer.sh"
python - <<'PY'
from pathlib import Path
import hashlib,json
import wildos_nav2_adapter.graph_path_bridge as bridge
import wildos_nav2_adapter.exploration_cost as cost
from ament_index_python.packages import get_package_prefix
r=Path('/home/ly/frontier_cost_repair_20261016');m=json.loads((r/'release.manifest.json').read_text());loaded={}
for module in (bridge,cost):
 p=Path(module.__file__);h=hashlib.sha256(p.read_bytes()).hexdigest();assert p.is_relative_to(r/'release_install')
 assert h==m['members']['nav2_adapter/wildos_nav2_adapter/'+p.name];loaded[str(p)]=h
assert get_package_prefix('graphnav_planner')==str(r/'install')
libs={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [r/'install/lib/libgraphnav_planner_component.so',Path('/home/ly/wildos-console/install/lib/libwildos_ackermann_smac.so'),Path('/home/ly/wildos-console/install/lib/libwildos_ackermann_rpp.so')]}
(r/'release_identity.json').write_text(json.dumps(dict(source=m['sha256'],installed=loaded,libraries=libs),indent=2))
print('Final compiled source, Python import and library identity verified')
PY
