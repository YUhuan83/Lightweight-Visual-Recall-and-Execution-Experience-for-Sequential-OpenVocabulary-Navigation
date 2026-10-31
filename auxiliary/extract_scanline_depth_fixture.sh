#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v46_approach_feedback/run_full.sh")
export PYTHONPATH="$base/v46_approach_feedback/candidate/wildos_port/repro_check/src/nebula2-wildos/visual_navigation:$PYTHONPATH"
/home/ly/beamng_ros2_0394_ws/.venv/bin/python "$base/extract_scanline_depth_fixture.py"
