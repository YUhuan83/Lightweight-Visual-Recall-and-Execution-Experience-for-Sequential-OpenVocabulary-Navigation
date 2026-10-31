#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
for version in v45_approach_contract v46_approach_feedback; do
  (
    source <(sed '/^exec /,$d' "$base/$version/run_full.sh")
    export ROS_DOMAIN_ID=217 ROS_LOCALHOST_ONLY=1
    for run in 1 2; do
      /home/ly/beamng_ros2_0394_ws/.venv/bin/python "$base/replay_approach_feedback.py" "$version" >"$base/$version/approach_feedback_replay_$run.log" 2>&1
      cp "$base/$version/approach_feedback_replay.json" "$base/$version/approach_feedback_replay_$run.json"
    done
    cmp "$base/$version/approach_feedback_replay_1.json" "$base/$version/approach_feedback_replay_2.json"
    tail -100 "$base/$version/approach_feedback_replay_2.log"
  )
done
