#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/wildos-console/install/setup.bash
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ly/wildos-console/bundle/nav2_adapter/config/fastdds_local_udp.xml
/home/ly/beamng_ros2_0394_ws/.venv/bin/python /home/ly/planner_wait_audit_20261015/extract_input.py
cmake -S /home/ly/planner_wait_audit_20261015 -B /home/ly/planner_wait_audit_20261015/build -DCMAKE_BUILD_TYPE=Release
cmake --build /home/ly/planner_wait_audit_20261015/build -j2
/home/ly/planner_wait_audit_20261015/build/candidate_probe --gtest_filter=ActualPlanner.SealedCandidateBudgetAudit
