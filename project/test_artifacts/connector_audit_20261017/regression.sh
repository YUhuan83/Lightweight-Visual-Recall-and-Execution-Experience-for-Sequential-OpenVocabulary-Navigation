#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/wildos-console/install/setup.bash
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
export ROS_DOMAIN_ID=175
cd /home/ly/connector_audit_20261017
python3 -m zipfile -e regression_sources.zip regression_sources
for mode in baseline candidate; do
 colcon --log-base regression_log_$mode build --base-paths regression_sources/$mode/ackermann_smac --build-base regression_build_$mode --install-base regression_install_$mode --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON >regression_build_$mode.log 2>&1
 (
 source regression_install_$mode/setup.bash
 if [ "$mode" = baseline ]; then
  set +e
  regression_build_$mode/wildos_ackermann_smac/test_planner --gtest_filter=ActualPlanner.RecordedCompleteRouteAvoidsFirstFeasibleFourCuspDetour >regression_baseline.log 2>&1
  code=$?
  echo "baseline_expected_failure_exit=$code"
  test "$code" = 1
 else
  colcon --log-base regression_test_log test --build-base regression_build_candidate --install-base regression_install_candidate --event-handlers console_direct+ >regression_candidate.log 2>&1
  colcon test-result --test-result-base regression_build_candidate --verbose
  regression_build_candidate/wildos_ackermann_smac/test_planner --gtest_filter=ActualPlanner.RecordedCompleteRouteAvoidsFirstFeasibleFourCuspDetour --gtest_repeat=3 >regression_repeat.log 2>&1
 fi
 )
done
