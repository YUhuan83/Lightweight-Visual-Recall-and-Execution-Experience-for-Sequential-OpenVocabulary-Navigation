#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v45_approach_contract/run_full.sh")
export ROS_DOMAIN_ID=216 ROS_LOCALHOST_ONLY=1
for run in 1 2; do
  for fixture in "$base/v45_approach_contract/instance_route_probe"/case_*.txt; do
    OBSERVATION_FIXTURE="$fixture" "$base/v33_integrated_progress/observation_pose_probe/probe" --gtest_filter=ActualPlanner.RecordedObservationPoses >"$fixture.run_$run.log" 2>&1
    grep OBS_RESULT "$fixture.run_$run.log"
  done
done
