#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v28_intent_handoff/run_full.sh")
python3 /home/ly/planning_phase_physical_20261018/prestart.py > "$base/confirmation_wait_precheck.json"
/home/ly/beamng_ros2_0394_ws/.venv/bin/python -c 'import json; p=json.load(open("/home/ly/execution_continuity_review_20261022/confirmation_wait_precheck.json")); assert not p["processes"] and not p["display148_lock"],p'
/home/ly/beamng_ros2_0394_ws/.venv/bin/python "$base/audit_confirmed_target_wait.py" "$base/v28_intent_handoff/full/trial_20261025_200844"
