#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/full_vehicle_acceptance_20261017/install/setup.bash
source /home/ly/planning_phase_physical_20261018/install/setup.bash
export ROS_DOMAIN_ID=183 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ly/planning_phase_physical_20261018/candidate/nav2_adapter/config/fastdds_local_udp.xml
cd /home/ly/vehicle_interface_audit_20261018
sha256sum /home/ly/full_vehicle_acceptance_20261017/install/wildos_ackermann_smac/lib/libwildos_ackermann_smac.so >library_identity.txt
cmake -S . -B build >configure_suffix.log 2>&1
cmake --build build --target replay_suffix -j 2 >build_suffix.log 2>&1
for index in 6 7; do
  for mode in smooth raw; do
    export REPLAY_INPUT="$PWD/case${index}.txt" REPLAY_FREE=1 REPLAY_SUFFIX_TRACE=1
    unset REPLAY_SUFFIX_RAW
    if [ "$mode" = raw ]; then export REPLAY_SUFFIX_RAW=1; fi
    build/replay_suffix --gtest_filter=ActualPlanner.FrozenVehicleInterfaceMatrix >"suffix_case${index}_${mode}.log" 2>&1
    grep SUFFIX_RESULT "suffix_case${index}_${mode}.log"
  done
done
