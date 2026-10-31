#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v32_retry_admission/run_full.sh")
/home/ly/beamng_ros2_0394_ws/.venv/bin/python "$base/replay_v32_bearings.py"
