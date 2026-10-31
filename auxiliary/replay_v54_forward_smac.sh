#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v54_confirmed_graph_handoff/run_full.sh")
root="$base/frontier_endpoint_contract_review"
python3 "$base/prepare_v54_forward_smac.py"
for run in 1 2; do
 OBSERVATION_FIXTURE="$root/forward_smac.txt" "$base/v33_integrated_progress/observation_pose_probe/probe" --gtest_filter=ActualPlanner.RecordedObservationPoses >"$root/forward_smac_$run.log" 2>&1
 grep OBS_RESULT "$root/forward_smac_$run.log"
done
