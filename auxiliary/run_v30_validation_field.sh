#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v30_confirmation_history/run_full.sh)
cd /home/ly/execution_continuity_review_20261022/v30_confirmation_history
export ROS_DOMAIN_ID=205 ROS_LOCALHOST_ONLY=1
export PYTHONPATH=$PWD/candidate/tools:$PWD/candidate/nav2_adapter/test:$PYTHONPATH
/home/ly/vision-cu128/bin/python /tmp/validate_v30_remote.py >perception_review.log 2>&1
python3 /tmp/run_v30_field.py >field_runner.log 2>&1
