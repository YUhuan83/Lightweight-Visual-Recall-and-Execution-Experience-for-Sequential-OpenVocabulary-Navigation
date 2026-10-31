#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v43_abort_recovery/run_full.sh")
cd "$base"
python3 audit_confirmed_progress_field.py v43_abort_recovery
python3 audit_vehicle_observation_closed.py v43_abort_recovery
python3 audit_full_path_closed.py v43_abort_recovery
python3 audit_retirement_boundary_closed.py v43_abort_recovery
