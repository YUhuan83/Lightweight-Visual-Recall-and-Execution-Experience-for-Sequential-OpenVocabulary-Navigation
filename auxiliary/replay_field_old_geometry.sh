#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v47_depth_geometry/run_full.sh")
export PYTHONPATH="$base/v47_depth_geometry/candidate/wildos_port/repro_check/src/nebula2-wildos/visual_navigation:$PYTHONPATH"
cd "$base"
for run in 1 2; do
 /home/ly/beamng_ros2_0394_ws/.venv/bin/python replay_field_old_geometry.py >"v47_depth_geometry/field_depth_old_geometry_$run.log" 2>&1
 cp v47_depth_geometry/field_depth_old_geometry.json "v47_depth_geometry/field_depth_old_geometry_$run.json"
done
cmp v47_depth_geometry/field_depth_old_geometry_1.json v47_depth_geometry/field_depth_old_geometry_2.json
cat v47_depth_geometry/field_depth_old_geometry_2.log
