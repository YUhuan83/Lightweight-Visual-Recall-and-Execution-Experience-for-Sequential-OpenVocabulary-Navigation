#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
for version in v45_approach_contract v46_approach_feedback; do
  (
    source <(sed '/^exec /,$d' "$base/$version/run_full.sh")
    export ROS_DOMAIN_ID=217 ROS_LOCALHOST_ONLY=1
    fixture="$base/$version/instance_route_probe/feedback_pose.txt"
    for run in 1 2; do
      OBSERVATION_FIXTURE="$fixture" "$base/v33_integrated_progress/observation_pose_probe/probe" --gtest_filter=ActualPlanner.RecordedObservationPoses >"$fixture.run_$run.log" 2>&1
      grep OBS_RESULT "$fixture.run_$run.log"
    done
  )
done
