#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v51_confirmed_action_handoff/run_full.sh")
cd "$base"
python3 audit_confirmed_progress_field.py v51_confirmed_action_handoff
python3 audit_vehicle_observation_closed.py v51_confirmed_action_handoff
python3 audit_full_path_closed.py v51_confirmed_action_handoff
python3 audit_retirement_boundary_closed.py v51_confirmed_action_handoff
python3 audit_range_hint_closed.py v51_confirmed_action_handoff
