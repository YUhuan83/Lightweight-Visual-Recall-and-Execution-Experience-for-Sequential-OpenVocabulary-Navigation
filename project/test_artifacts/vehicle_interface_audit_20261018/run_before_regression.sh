#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/full_vehicle_acceptance_20261017/install/setup.bash
source /home/ly/planning_phase_physical_20261018/install/setup.bash
export ROS_DOMAIN_ID=183 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ly/planning_phase_physical_20261018/candidate/nav2_adapter/config/fastdds_local_udp.xml
cd /home/ly/vehicle_interface_audit_20261018
cmake --build build --target replay_suffix -j 2 >build_before_assert.log 2>&1
unset REPLAY_TRACE REPLAY_SUFFIX_TRACE REPLAY_NATIVE
export REPLAY_REQUIRE_SUCCESS=1 REPLAY_FREE=1
for index in 6 7; do
  export REPLAY_INPUT="$PWD/case${index}.txt" REPLAY_OUTPUT="$PWD/regression_before_case${index}.csv"
  set +e
  build/replay_suffix --gtest_filter=ActualPlanner.FrozenVehicleInterfaceMatrix >"regression_before_case${index}.log" 2>&1
  code=$?
  set -e
  test "$code" = 1
  grep -E 'MATRIX_RESULT|FAILED' "regression_before_case${index}.log"
done
