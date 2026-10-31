#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v48_range_candidate/run_full.sh")
for version in v47_depth_geometry v48_range_candidate; do
 export PYTHONPATH="$base/$version/candidate/official_adapter:$base/$version/candidate/nav2_adapter:$base/$version/candidate/wildos_port/repro_check/src/nebula2-wildos/visual_navigation:$PYTHONPATH"
 for run in 1 2; do
  /home/ly/beamng_ros2_0394_ws/.venv/bin/python "$base/replay_range_candidate.py" "$version" >"$base/$version/range_candidate_replay_$run.log" 2>&1
  cp "$base/$version/range_candidate_replay.json" "$base/$version/range_candidate_replay_$run.json"
 done
 cmp "$base/$version/range_candidate_replay_1.json" "$base/$version/range_candidate_replay_2.json"
 cat "$base/$version/range_candidate_replay_2.log"
done
