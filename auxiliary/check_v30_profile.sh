#!/usr/bin/env bash
set -eo pipefail
root=/home/ly/execution_continuity_review_20261022/v30_confirmation_history
source <(sed '/^exec /,$d' "$root/run_full.sh")
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=0
timeout 15s ros2 param get /beamng_vehicle_goal_standoff confirmation_window_sec | tee "$root/actual_confirmation_window.txt"
