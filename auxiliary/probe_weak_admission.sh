#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v40_observation_heading/run_full.sh")
export ROS_DOMAIN_ID=211 ROS_LOCALHOST_ONLY=1
python3 "$base/extract_weak_admission.py" >"$base/v40_observation_heading/weak_admission_inputs.log" 2>&1
for repeat in 1 2; do
  for fixture in "$base/v40_observation_heading/weak_admission_probe"/case_*.txt; do
    OBSERVATION_FIXTURE="$fixture" "$base/v33_integrated_progress/observation_pose_probe/probe" --gtest_filter=ActualPlanner.RecordedObservationPoses > "$fixture.run_$repeat.log" 2>&1
    grep OBS_RESULT "$fixture.run_$repeat.log"
  done
done
