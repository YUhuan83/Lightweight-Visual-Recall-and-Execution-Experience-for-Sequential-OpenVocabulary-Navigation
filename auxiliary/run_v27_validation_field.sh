#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v27_fresh_bearing/run_full.sh)
cd /home/ly/execution_continuity_review_20261022/v27_fresh_bearing
export ROS_DOMAIN_ID=205 ROS_LOCALHOST_ONLY=1
export PYTHONPATH=$PWD/candidate/tools:$PWD/candidate/nav2_adapter/test:$PYTHONPATH
/home/ly/beamng_ros2_0394_ws/.venv/bin/python real_ros_review.py >real_ros_review.log 2>&1
python3 /tmp/run_v27_field.py >field_runner.log 2>&1
