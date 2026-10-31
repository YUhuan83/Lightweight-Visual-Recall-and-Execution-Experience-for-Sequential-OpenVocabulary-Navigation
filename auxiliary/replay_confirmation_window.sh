#!/usr/bin/env bash
set -eo pipefail
python3 /home/ly/planning_phase_physical_20261018/prestart.py
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v29_range_consistency_r2/run_full.sh)
export ROS_DOMAIN_ID=207 ROS_LOCALHOST_ONLY=1
/home/ly/beamng_ros2_0394_ws/.venv/bin/python /tmp/replay_confirmation_window.py
