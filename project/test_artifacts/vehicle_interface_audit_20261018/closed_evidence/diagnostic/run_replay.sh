#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/full_vehicle_acceptance_20261017/install/setup.bash
source /home/ly/planning_phase_physical_20261018/install/setup.bash
export ROS_DOMAIN_ID=183 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ly/planning_phase_physical_20261018/candidate/nav2_adapter/config/fastdds_local_udp.xml
cd /home/ly/vehicle_interface_audit_20261018
/home/ly/beamng_ros2_0394_ws/.venv/bin/python export_planner_cases.py >export.log
cmake -S . -B build >configure.log 2>&1
cmake --build build -j 2 >build.log 2>&1
for index in 0 6 7; do
  for map in recorded free; do
    for plugin in wrapped native; do
      unset REPLAY_FREE REPLAY_NATIVE REPLAY_RADIUS8
      export REPLAY_INPUT="$PWD/case${index}.txt" REPLAY_OUTPUT="$PWD/case${index}_${map}_${plugin}.csv"
      if [ "$map" = free ]; then export REPLAY_FREE=1; fi
      if [ "$plugin" = native ]; then export REPLAY_NATIVE=1; fi
      build/replay_matrix --gtest_filter=ActualPlanner.FrozenVehicleInterfaceMatrix >"case${index}_${map}_${plugin}.log" 2>&1
      grep MATRIX_RESULT "case${index}_${map}_${plugin}.log"
    done
  done
done
