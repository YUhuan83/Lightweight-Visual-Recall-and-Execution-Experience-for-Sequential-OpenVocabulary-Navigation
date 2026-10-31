#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/town_memory_validation_20261014/tracking_contract_candidate/install/setup.bash
source /home/ly/town_memory_validation_20261014/curvature_contract_candidate1/install/setup.bash
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1
ROOT=/home/ly/town_memory_validation_20261014/dds_transport_candidate/geometry_replay1
cd "$ROOT"
cmake -S . -B build_repaired >build_repaired.log 2>&1
cmake --build build_repaired -j4 >>build_repaired.log 2>&1
mkdir repaired_outputs
cp input0.txt input1.txt input3.txt repaired_outputs/
cd repaired_outputs
for i in 0 1 3; do
  for map in recorded free; do
    for mode in raw smooth; do
      unset GEOMETRY_SMOOTH GEOMETRY_FREE
      export GEOMETRY_CASE=$i
      if [ "$mode" = smooth ]; then export GEOMETRY_SMOOTH=1; fi
      if [ "$map" = free ]; then export GEOMETRY_FREE=1; fi
      "$ROOT/build_repaired/recorded_planner" --gtest_filter=ActualPlanner.FullTrialRecordedGeometryAudit >"case${i}_${map}_${mode}.log" 2>&1
      grep -E 'GEOMETRY_PLAN|smoothed path rejected' "case${i}_${map}_${mode}.log"
    done
  done
done
