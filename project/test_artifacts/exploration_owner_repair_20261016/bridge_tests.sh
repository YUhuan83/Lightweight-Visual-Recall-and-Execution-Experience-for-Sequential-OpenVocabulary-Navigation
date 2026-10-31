#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/town_memory_validation_20261014/curvature_contract_candidate1/install/setup.bash
source /home/ly/wildos-console/install/setup.bash
source /home/ly/exploration_owner_repair_20261016/install/setup.bash
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ly/wildos-console/bundle/nav2_adapter/config/fastdds_local_udp.xml
export PYTHONPATH=/home/ly/exploration_owner_repair_20261016/bundle/nav2_adapter:/home/ly/exploration_owner_repair_20261016/bundle/nav2_adapter/test:$PYTHONPATH
/home/ly/beamng_ros2_0394_ws/.venv/bin/python -m unittest test_graph_path_bridge_routing.ExplorationTargetHandoffTests -v
/home/ly/beamng_ros2_0394_ws/.venv/bin/python - <<'PY'
import rclpy, unittest
rclpy.init()
try:
 suite=unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromName(name) for name in ['test_planner_execution_ownership_ros','test_tracking_handoff_ros','test_short_endpoint_handoff_ros'])
 result=unittest.TextTestRunner(verbosity=2).run(suite)
 assert result.wasSuccessful()
finally:rclpy.shutdown()
PY
