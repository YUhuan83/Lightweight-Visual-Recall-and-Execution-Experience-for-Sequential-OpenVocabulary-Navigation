#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v9/run_full.sh)
source /home/ly/execution_continuity_review_20261022/v22_odometry_ingress/install/local_setup.bash
for trial in /home/ly/execution_continuity_review_20261022/v22_odometry_ingress/full/trial_20261024_165941 /home/ly/execution_continuity_review_20261022/v22_odometry_ingress/full/trial_20261024_170417; do
  goal=$(find "$trial/sessions" -mindepth 2 -maxdepth 2 -type d -name goal_001)
  for repeat in 1 2; do
    /home/ly/beamng_ros2_0394_ws/.venv/bin/python /tmp/audit_command_gaps.py "$goal" > "$trial/command_gaps_repeat_$repeat.json"
    sha256sum "$trial/command_gaps_repeat_$repeat.json"
  done
done
