#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v34_vehicle_observation/run_full.sh")
/home/ly/beamng_ros2_0394_ws/.venv/bin/python "$base/audit_confirmed_progress_field.py" v34_vehicle_observation
/home/ly/beamng_ros2_0394_ws/.venv/bin/python "$base/audit_vehicle_observation_closed.py"
