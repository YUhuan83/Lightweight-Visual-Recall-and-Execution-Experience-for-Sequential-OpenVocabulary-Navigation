#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v28_intent_handoff/run_full.sh)
cd /home/ly/execution_continuity_review_20261022/v28_intent_handoff
export ROS_DOMAIN_ID=205 ROS_LOCALHOST_ONLY=1
/home/ly/vision-cu128/bin/python /tmp/validate_v28_remote.py >perception_review.log 2>&1
/home/ly/beamng_ros2_0394_ws/.venv/bin/python /tmp/check_v28_sources.py
python3 /tmp/run_v28_field.py >field_runner.log 2>&1
