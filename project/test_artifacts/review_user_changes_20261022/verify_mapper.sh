#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v1/run_full.sh)
source /home/ly/execution_continuity_review_20261022/v2/install/local_setup.bash
cd /home/ly/execution_continuity_review_20261022/v2
export ROS_DOMAIN_ID=205 ROS_LOCALHOST_ONLY=1
build/graphnav_mapper/graphnav_mapper_test_mapper >mapper_tests_after.log 2>&1
/usr/bin/python3 /home/ly/execution_continuity_review_20261022/incoming/compile_replay.py >mapper_replay_after.log 2>&1
