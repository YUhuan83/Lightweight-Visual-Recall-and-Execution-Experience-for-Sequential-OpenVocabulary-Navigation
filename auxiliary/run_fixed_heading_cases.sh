#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v9/run_full.sh)
source /home/ly/execution_continuity_review_20261022/v22_odometry_ingress/install/local_setup.bash
export ROS_DOMAIN_ID=208 ROS_LOCALHOST_ONLY=1
python3 /tmp/compile_fixed_heading_cases.py
for repeat in 1 2; do
  /home/ly/beamng_ros2_0394_ws/.venv/bin/python /tmp/extract_fixed_heading_cases.py
  for fixture in /home/ly/execution_continuity_review_20261022/fixed_heading_replay_20261024/case_*.txt; do
    HEADING_FIXTURE="$fixture" /home/ly/execution_continuity_review_20261022/fixed_heading_replay_20261024/heading_replay --gtest_filter=ActualPlanner.FixedHeadingCounterfactual > "$fixture.run_$repeat.log" 2>&1
    grep HEADING_RESULT "$fixture.run_$repeat.log"
  done
done
