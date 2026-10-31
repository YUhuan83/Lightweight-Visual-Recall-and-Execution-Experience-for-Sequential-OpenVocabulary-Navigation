#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_repair_20261021/reachable_observation_v11/run_full.sh)
/home/ly/beamng_ros2_0394_ws/.venv/bin/python /home/ly/execution_continuity_repair_20261021/reachable_observation_incoming/extract_path_geometry.py
