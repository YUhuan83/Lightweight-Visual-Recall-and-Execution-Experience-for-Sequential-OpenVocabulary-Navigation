#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v34_vehicle_observation/run_full.sh")
export ROS_DOMAIN_ID=215 ROS_LOCALHOST_ONLY=1
for repeat in 1 2; do
  /home/ly/beamng_ros2_0394_ws/.venv/bin/python "$base/replay_confirmed_vehicle_steps.py" "confirmed_step_replay_$repeat.json" > "$base/v34_vehicle_observation/confirmed_step_replay_$repeat.log" 2>&1
  tail -30 "$base/v34_vehicle_observation/confirmed_step_replay_$repeat.log"
  for fixture in "$base/v34_vehicle_observation/confirmed_step_probe"/case_*.txt; do
    test -f "$fixture" || continue
    OBSERVATION_FIXTURE="$fixture" "$base/v33_integrated_progress/observation_pose_probe/probe" --gtest_filter=ActualPlanner.RecordedObservationPoses > "$fixture.run_$repeat.log" 2>&1
    grep OBS_RESULT "$fixture.run_$repeat.log"
  done
done
