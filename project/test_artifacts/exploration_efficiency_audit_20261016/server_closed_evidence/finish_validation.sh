#!/usr/bin/env bash
set -eo pipefail
source /home/ly/exploration_efficiency_audit_20261016/env.sh
R=/home/ly/exploration_efficiency_audit_20261016
export PYTHONPATH="$R/candidate/nav2_adapter/test:$PYTHONPATH"
# Baseline uses the same tests with currently deployed 11af production.
/home/ly/beamng_ros2_0394_ws/.venv/bin/python - <<'PY' > "$R/legacy_typed_baseline.log" 2>&1
import rclpy,unittest
rclpy.init()
try:
 s=unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromName(n) for n in ['test_execution_mechanism_ros','test_planner_execution_ownership_ros','test_tracking_handoff_ros','test_short_endpoint_handoff_ros'])
 result=unittest.TextTestRunner(verbosity=2).run(s)
 print('BASELINE_FAILURES',len(result.failures),'BASELINE_ERRORS',len(result.errors))
finally:rclpy.shutdown()
PY
source "$R/install/setup.bash"
/home/ly/beamng_ros2_0394_ws/.venv/bin/python - <<'PY'
import rclpy,unittest
rclpy.init()
try:
 names=['test_planner_execution_ownership_ros','test_tracking_handoff_ros','test_short_endpoint_handoff_ros']
 import test_execution_mechanism_ros as m
 names += ['test_execution_mechanism_ros.ExecutionMechanismTests.'+n for n in unittest.defaultTestLoader.getTestCaseNames(m.ExecutionMechanismTests) if any(x in n for x in ('escape','recover'))]
 s=unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromName(n) for n in names)
 result=unittest.TextTestRunner(verbosity=2).run(s);assert result.wasSuccessful()
finally:rclpy.shutdown()
PY
/home/ly/beamng_ros2_0394_ws/.venv/bin/python /home/ly/exploration_owner_repair_20261016/test_graphnav_owner_ros.py
