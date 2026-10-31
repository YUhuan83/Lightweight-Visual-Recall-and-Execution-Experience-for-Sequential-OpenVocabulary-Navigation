#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v46_approach_feedback/run_full.sh")
export PYTHONPATH="$base/v46_approach_feedback/candidate/wildos_port/repro_check/src/nebula2-wildos/visual_navigation:$PYTHONPATH"
cd "$base"
for mode in prefilter dense; do
  for run in 1 2; do
    /home/ly/beamng_ros2_0394_ws/.venv/bin/python replay_localization_sampling.py "$mode" >"v46_approach_feedback/localization_sampling_${mode}_$run.log" 2>&1
    cp "v46_approach_feedback/localization_sampling_$mode.json" "v46_approach_feedback/localization_sampling_${mode}_$run.json"
  done
  cmp "v46_approach_feedback/localization_sampling_${mode}_1.json" "v46_approach_feedback/localization_sampling_${mode}_2.json"
  cat "v46_approach_feedback/localization_sampling_${mode}_2.log"
done
