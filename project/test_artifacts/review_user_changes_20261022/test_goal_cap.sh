#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v2/run_full.sh)
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
cd /home/ly/execution_continuity_review_20261022/goal_cap_probe
export ROS_DOMAIN_ID=206 ROS_LOCALHOST_ONLY=1
colcon --log-base build_log build --base-paths ackermann_smac --packages-select wildos_ackermann_smac --build-base build --install-base install >build.log 2>&1
source install/local_setup.bash
build/wildos_ackermann_smac/test_planner --gtest_filter='ActualPlanner.LongerStraightGoalDoesNotInheritStartConnectorLengthCap' >counterexample_before.log 2>&1
