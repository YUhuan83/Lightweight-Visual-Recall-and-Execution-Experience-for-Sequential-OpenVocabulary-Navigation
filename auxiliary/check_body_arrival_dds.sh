#!/usr/bin/env bash
set -eo pipefail
python3 /home/ly/planning_phase_physical_20261018/prestart.py
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v26_body_arrival/run_full.sh)
export ROS_DOMAIN_ID=206 ROS_LOCALHOST_ONLY=1
export PYTHONPATH=/home/ly/execution_continuity_review_20261022/v26_body_arrival/candidate/tools:$PYTHONPATH
/home/ly/beamng_ros2_0394_ws/.venv/bin/python /tmp/check_body_arrival_dds.py
