#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v9/run_full.sh)
source /home/ly/execution_continuity_review_20261022/v18_comparison_bound/install/local_setup.bash
trial=$1
goal=$(find "$trial/sessions" -maxdepth 2 -type d -name goal_001)
test -n "$goal"
/home/ly/beamng_ros2_0394_ws/.venv/bin/python /tmp/collect_v17_trial.py "$trial" > "$trial/analysis.log"
/home/ly/beamng_ros2_0394_ws/.venv/bin/python /tmp/audit_v17_stale_command.py "$goal" > "$trial/stale_command.json"
/home/ly/beamng_ros2_0394_ws/.venv/bin/python /tmp/audit_command_gaps.py "$goal" > "$trial/command_gaps.json"
