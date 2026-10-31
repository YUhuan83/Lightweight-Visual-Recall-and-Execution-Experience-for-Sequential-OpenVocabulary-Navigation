#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v46_approach_feedback/run_full.sh")
export PYTHONPATH="$base/v46_approach_feedback/candidate/wildos_port/repro_check/src/nebula2-wildos/visual_navigation:$PYTHONPATH"
cd "$base"
for run in 1 2; do
  /home/ly/beamng_ros2_0394_ws/.venv/bin/python replay_v46_localization.py >"v46_approach_feedback/localization_failure_replay_$run.log" 2>&1
  cp v46_approach_feedback/localization_failure_replay.json "v46_approach_feedback/localization_failure_replay_$run.json"
done
cmp v46_approach_feedback/localization_failure_replay_1.json v46_approach_feedback/localization_failure_replay_2.json
cat v46_approach_feedback/localization_failure_replay_2.log
