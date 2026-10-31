#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v36_weak_observation/run_full.sh")
for version in v35_confirmed_vehicle v36_weak_observation; do
  for repeat in 1 2; do
    /home/ly/beamng_ros2_0394_ws/.venv/bin/python "$base/audit_motion_section_contract.py" "$version" "motion_section_audit_$repeat.json"
  done
done
