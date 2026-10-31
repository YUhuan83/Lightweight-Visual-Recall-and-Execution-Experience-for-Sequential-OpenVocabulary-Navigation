#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v49_localization_range/run_full.sh")
export ROS_DOMAIN_ID=214 ROS_LOCALHOST_ONLY=1
for run in 1 2; do
 /home/ly/beamng_ros2_0394_ws/.venv/bin/python "$base/audit_confirmed_window.py" >"$base/v49_localization_range/confirmed_window_audit_$run.log" 2>&1
 cp "$base/v49_localization_range/confirmed_window_audit.json" "$base/v49_localization_range/confirmed_window_audit_$run.json"
done
cmp "$base/v49_localization_range/confirmed_window_audit_1.json" "$base/v49_localization_range/confirmed_window_audit_2.json"
cat "$base/v49_localization_range/confirmed_window_audit_2.log"
