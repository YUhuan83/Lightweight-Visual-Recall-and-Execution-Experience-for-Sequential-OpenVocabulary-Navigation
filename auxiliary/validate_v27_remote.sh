#!/usr/bin/env bash
set -eo pipefail
python3 /home/ly/planning_phase_physical_20261018/prestart.py
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v27_fresh_bearing/run_full.sh)
cd /home/ly/execution_continuity_review_20261022/v27_fresh_bearing
export ROS_DOMAIN_ID=205 ROS_LOCALHOST_ONLY=1
/home/ly/vision-cu128/bin/python /tmp/validate_v27_remote.py >perception_review.log 2>&1
cat perception_report.json
