#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v47_depth_geometry/run_full.sh")
export PYTHONPATH="$base/v47_depth_geometry/candidate/wildos_port/repro_check/src/nebula2-wildos/visual_navigation:$PYTHONPATH"
cd "$base"
for run in 1 2; do
  /home/ly/beamng_ros2_0394_ws/.venv/bin/python replay_old_depth_regression.py >"v47_depth_geometry/old_depth_regression_$run.log" 2>&1
  cp v47_depth_geometry/old_depth_regression.json "v47_depth_geometry/old_depth_regression_$run.json"
  /home/ly/beamng_ros2_0394_ws/.venv/bin/python replay_new_depth_geometry.py >"v47_depth_geometry/new_depth_replay_$run.log" 2>&1
  cp v47_depth_geometry/new_depth_replay.json "v47_depth_geometry/new_depth_replay_$run.json"
done
cmp v47_depth_geometry/old_depth_regression_1.json v47_depth_geometry/old_depth_regression_2.json
cmp v47_depth_geometry/new_depth_replay_1.json v47_depth_geometry/new_depth_replay_2.json
cat v47_depth_geometry/old_depth_regression_2.log
cat v47_depth_geometry/new_depth_replay_2.log
