#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
root=/home/ly/execution_continuity_review_20261022
for trial in "$root/v19_dispatch_reuse/full/trial_20261024_153126" \
             "$root/v19_dispatch_reuse/full/trial_20261024_153959" \
             "$root/v20_stop_plan_overlap/full/trial_20261024_153554" \
             "$root/v20_stop_plan_overlap/full/trial_20261024_154411"; do
  goal=$(find "$trial/sessions" -maxdepth 2 -type d -name goal_001)
  test -f "$goal/stop.json"
  /home/ly/beamng_ros2_0394_ws/.venv/bin/python /tmp/audit_command_gaps.py "$goal" > "$trial/command_gaps_repeat_1.json"
  /home/ly/beamng_ros2_0394_ws/.venv/bin/python /tmp/audit_command_gaps.py "$goal" > "$trial/command_gaps_repeat_2.json"
  cmp "$trial/command_gaps_repeat_1.json" "$trial/command_gaps_repeat_2.json"
  sha256sum "$trial/command_gaps_repeat_1.json"
done
