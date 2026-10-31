#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v40_observation_heading/run_full.sh)
cd /home/ly/execution_continuity_review_20261022
python3 audit_confirmed_progress_field.py v40_observation_heading
python3 audit_vehicle_observation_closed.py v40_observation_heading
python3 audit_full_path_closed.py v40_observation_heading
