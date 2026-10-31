#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/wildos-console/install/setup.bash
source /home/ly/visible_pursuit_repair_20261017/install/setup.bash
export ROS_DOMAIN_ID=175
cd /home/ly/connector_audit_20261017
python3 -m zipfile -e diagnostic.zip diagnostic
cmake -S diagnostic -B build > configure.log 2>&1
cmake --build build -j2 > build.log 2>&1
for index in $(seq 0 18); do
  GEOMETRY_CASE=$index GEOMETRY_SMOOTH=1 ./build/recorded_planner --gtest_filter=ActualPlanner.FullTrialRecordedGeometryAudit > "case${index}.log" 2>&1
  grep -E 'PENALTIES|CONNECTOR|CURVE|GEOMETRY_PLAN' "case${index}.log"
done
