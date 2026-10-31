#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v32_retry_admission/run_full.sh")
/home/ly/beamng_ros2_0394_ws/.venv/bin/python "$base/audit_confirmed_progress_field.py" v32_retry_admission
/home/ly/beamng_ros2_0394_ws/.venv/bin/python "$base/audit_abort_route_retry.py" v32_retry_admission
