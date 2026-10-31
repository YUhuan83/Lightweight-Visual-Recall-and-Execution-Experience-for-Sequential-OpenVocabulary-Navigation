#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v33_integrated_progress/run_full.sh")
export ROS_DOMAIN_ID=213 ROS_LOCALHOST_ONLY=1
/home/ly/beamng_ros2_0394_ws/.venv/bin/python "$base/extract_vehicle_observation_cases.py"
python3 "$base/compile_vehicle_observation.py"
for repeat in 1 2; do
  for fixture in "$base/v33_integrated_progress/observation_pose_probe"/case_*.txt; do
    OBSERVATION_FIXTURE="$fixture" "$base/v33_integrated_progress/observation_pose_probe/probe" --gtest_filter=ActualPlanner.RecordedObservationPoses > "$fixture.run_$repeat.log" 2>&1
    grep OBS_RESULT "$fixture.run_$repeat.log"
  done
done
