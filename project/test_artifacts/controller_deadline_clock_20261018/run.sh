set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/controller_source_field_20261018/install/setup.bash
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
export ROS_DOMAIN_ID=182 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
cd /home/ly/controller_deadline_clock_20261018
python3 -m zipfile -e candidate.zip candidate
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/candidate/nav2_adapter/config/fastdds_local_udp.xml"
set +e
PYTHONPATH="$PWD/candidate/nav2_adapter/test:$PYTHONPATH" python3 - <<'PY' >before_regression.log 2>&1
import unittest,rclpy
rclpy.init()
try:
 result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromName('test_controller_source_ros.ControllerSourceTests.test_scheduling_pause_between_source_check_and_issue_brakes_without_exception'))
finally:rclpy.shutdown()
raise SystemExit(not result.wasSuccessful())
PY
status=$?
set -e
test "$status" = 1
grep -q 'source_lifetime' before_regression.log
colcon --log-base build_log build --base-paths candidate/nav2_adapter --build-base build --install-base install >build.log 2>&1
source install/setup.bash
python3 server_callbacks.py >server_callbacks.log 2>&1
export ROS_DOMAIN_ID=181
python3 dds_chain.py >dds_chain.log 2>&1
tail -2 server_callbacks.log
tail -24 dds_chain.log
