#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v47_depth_geometry/run_full.sh")
export PYTHONPATH="$base/v47_depth_geometry/candidate/tools:$base/v47_depth_geometry/candidate/wildos_port/repro_check/src/nebula2-wildos/visual_navigation:$PYTHONPATH"
exec /home/ly/beamng_ros2_0394_ws/.venv/bin/python "$base/probe_static_lidar_coverage.py"
