#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v37_full_path/run_full.sh)
cd /home/ly/execution_continuity_review_20261022
python3 audit_confirmed_progress_field.py v37_full_path
python3 audit_vehicle_observation_closed.py v37_full_path
python3 audit_full_path_closed.py
