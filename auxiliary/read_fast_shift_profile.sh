#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v9/run_full.sh)
source /home/ly/execution_continuity_review_20261022/v25_fast_shift/install/local_setup.bash
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1
/home/ly/beamng_ros2_0394_ws/.venv/bin/python /tmp/read_fast_shift_profile.py
