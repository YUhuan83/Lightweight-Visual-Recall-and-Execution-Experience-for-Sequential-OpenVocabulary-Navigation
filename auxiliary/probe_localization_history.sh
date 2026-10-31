#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v46_approach_feedback/run_full.sh")
export PYTHONPATH="$base/v46_approach_feedback/candidate/wildos_port/repro_check/src/nebula2-wildos/visual_navigation:$PYTHONPATH"
cd "$base"
for history in 5 15 30; do
  /home/ly/beamng_ros2_0394_ws/.venv/bin/python probe_localization_history.py "$history" >"v46_approach_feedback/localization_history_$history.log" 2>&1
  cat "v46_approach_feedback/localization_history_$history.log"
done
