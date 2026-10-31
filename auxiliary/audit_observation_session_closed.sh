#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v39_observation_session/run_full.sh)
cd /home/ly/execution_continuity_review_20261022
python3 audit_confirmed_progress_field.py v39_observation_session
python3 audit_vehicle_observation_closed.py v39_observation_session
python3 audit_full_path_closed.py v39_observation_session
