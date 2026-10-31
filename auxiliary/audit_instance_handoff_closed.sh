#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v44_instance_handoff/run_full.sh")
cd "$base"
python3 audit_confirmed_progress_field.py v44_instance_handoff
python3 audit_vehicle_observation_closed.py v44_instance_handoff
python3 audit_full_path_closed.py v44_instance_handoff
python3 audit_retirement_boundary_closed.py v44_instance_handoff
