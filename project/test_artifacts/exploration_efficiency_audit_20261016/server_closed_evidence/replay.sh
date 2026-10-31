#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/town_memory_validation_20261014/curvature_contract_candidate1/install/setup.bash
source /home/ly/wildos-console/install/setup.bash
source /home/ly/exploration_owner_repair_20261016/final_install/setup.bash
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ly/wildos-console/bundle/nav2_adapter/config/fastdds_local_udp.xml
R=/home/ly/exploration_efficiency_audit_20261016
/home/ly/beamng_ros2_0394_ws/.venv/bin/python "$R/extract_cases.py" > "$R/extraction.log"
cd "$R"
export GEOMETRY_SMOOTH=1
unset GEOMETRY_FREE
for case_index in $(seq 0 9); do
 export GEOMETRY_CASE=$case_index
 /home/ly/town_memory_validation_20261014/dds_transport_candidate/geometry_replay1/build_repaired/recorded_planner --gtest_filter=ActualPlanner.FullTrialRecordedGeometryAudit > "case${case_index}.log" 2>&1
 grep 'GEOMETRY_PLAN' "case${case_index}.log"
done
