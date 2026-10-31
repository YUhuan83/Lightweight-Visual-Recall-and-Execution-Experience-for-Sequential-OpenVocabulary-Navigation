#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v35_confirmed_vehicle/run_full.sh")
export ROS_DOMAIN_ID=216 ROS_LOCALHOST_ONLY=1
for repeat in 1 2; do
  /home/ly/beamng_ros2_0394_ws/.venv/bin/python "$base/replay_v35_confirmed_vehicle.py" "confirmed_replay_$repeat.json" > "$base/v35_confirmed_vehicle/confirmed_replay_$repeat.log" 2>&1
  tail -30 "$base/v35_confirmed_vehicle/confirmed_replay_$repeat.log"
  for fixture in "$base/v35_confirmed_vehicle/confirmed_step_probe"/case_*.txt; do
    test -f "$fixture" || continue
    OBSERVATION_FIXTURE="$fixture" "$base/v33_integrated_progress/observation_pose_probe/probe" --gtest_filter=ActualPlanner.RecordedObservationPoses > "$fixture.run_$repeat.log" 2>&1
    grep OBS_RESULT "$fixture.run_$repeat.log"
  done
done
