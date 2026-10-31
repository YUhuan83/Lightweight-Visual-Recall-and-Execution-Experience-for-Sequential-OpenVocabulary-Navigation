#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v35_confirmed_vehicle/run_full.sh")
/home/ly/beamng_ros2_0394_ws/.venv/bin/python "$base/export_weak_shunt_cases.py"
