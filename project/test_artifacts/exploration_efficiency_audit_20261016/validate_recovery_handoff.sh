#!/usr/bin/env bash
set -eo pipefail
R=/home/ly/planning_recovery_handoff_20261016
source /home/ly/exploration_efficiency_audit_20261016/env.sh
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
python - <<'PY'
import hashlib,json,zipfile
from pathlib import Path
r=Path('/home/ly/planning_recovery_handoff_20261016')
m=json.loads((r/'official_wildos_beamng_bundle.manifest.json').read_text())
assert m['sha256']=='415c0bef49adf99f0fd7c64de75d7210973715090c6002fe8742064b92f7dde3'
a=r/'official_wildos_beamng_bundle.zip'
assert hashlib.sha256(a.read_bytes()).hexdigest()==m['sha256']
b=r/'candidate';b.mkdir(exist_ok=False)
with zipfile.ZipFile(a) as z:
 assert set(z.namelist())==set(m['members'])
 for n,h in m['members'].items():
  p=b/n;assert p.resolve().is_relative_to(b.resolve())
  v=z.read(n);assert hashlib.sha256(v).hexdigest()==h
  p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(v)
old=json.loads(Path('/home/ly/wildos-console/source_identity.json').read_text())
diff={n for n,h in m['members'].items() if old['members'].get(n)!=h}
assert diff=={'nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py','nav2_adapter/wildos_nav2_adapter/execution_recovery.py','nav2_adapter/test/test_graph_path_bridge_routing.py','nav2_adapter/test/test_navigation_transactions_ros.py','nav2_adapter/wildos_nav2_adapter/navigation_transactions.py'},diff
(r/'candidate_delta.json').write_text(json.dumps(dict(source=m['sha256'],base=old['sha256'],changed=sorted(diff)),indent=2))
PY
colcon --log-base "$R/build_logs" build --build-base "$R/build" --install-base "$R/install" --merge-install --base-paths "$R/candidate/nav2_adapter" --packages-select wildos_nav2_adapter
source "$R/install/setup.bash"
export PYTHONPATH="$R/candidate/nav2_adapter/test:$PYTHONPATH"
python -m unittest test_graph_path_bridge_routing.ExplorationTargetHandoffTests -v
python - <<'PY'
import rclpy,unittest
rclpy.init()
try:
 names=['test_planner_execution_ownership_ros','test_tracking_handoff_ros','test_short_endpoint_handoff_ros']+['test_navigation_transactions_ros.NavigationTransactionTests.'+n for n in ('test_escape_waits_for_old_terminal_result_and_settled_stop','test_new_query_revokes_escape_intent_during_cancellation_wait','test_expired_or_moved_escape_intent_falls_back_after_stop')]
 import test_execution_mechanism_ros as m
 names += ['test_execution_mechanism_ros.ExecutionMechanismTests.'+n for n in unittest.defaultTestLoader.getTestCaseNames(m.ExecutionMechanismTests) if any(x in n for x in ('escape','recover'))]
 result=unittest.TextTestRunner(verbosity=2).run(unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromName(n) for n in names))
 assert result.wasSuccessful()
finally:rclpy.shutdown()
PY
python /home/ly/exploration_owner_repair_20261016/test_graphnav_owner_ros.py
python - <<'PY'
import hashlib,json
from pathlib import Path
import wildos_nav2_adapter.graph_path_bridge as bridge
import wildos_nav2_adapter.execution_recovery as recovery
r=Path('/home/ly/planning_recovery_handoff_20261016');m=json.loads((r/'official_wildos_beamng_bundle.manifest.json').read_text())
loaded={}
for module in (bridge,recovery):
 p=Path(module.__file__);h=hashlib.sha256(p.read_bytes()).hexdigest()
 assert p.is_relative_to(r/'install')
 assert h==m['members']['nav2_adapter/wildos_nav2_adapter/'+p.name]
 loaded[str(p)]=h
production=Path('/home/ly/wildos-console');old=json.loads((production/'source_identity.json').read_text())
for n,h in old['members'].items():assert hashlib.sha256((production/'bundle'/n).read_bytes()).hexdigest()==h,n
(r/'validation_identity.json').write_text(json.dumps(dict(candidate=m['sha256'],production=old['sha256'],production_unchanged=True,installed=loaded,vehicle_run=False),indent=2))
print('Candidate installed source identity and unchanged production verified')
PY
