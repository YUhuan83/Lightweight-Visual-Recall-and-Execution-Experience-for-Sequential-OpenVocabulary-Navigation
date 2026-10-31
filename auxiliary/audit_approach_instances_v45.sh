#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v45_approach_contract/run_full.sh")
cd "$base"
for run in 1 2; do
  /home/ly/beamng_ros2_0394_ws/.venv/bin/python audit_approach_instances.py v45_approach_contract >"v45_approach_contract/approach_instance_$run.log"
  cp v45_approach_contract/approach_instance_audit.json "v45_approach_contract/approach_instance_audit_$run.json"
done
cmp v45_approach_contract/approach_instance_audit_1.json v45_approach_contract/approach_instance_audit_2.json
cat v45_approach_contract/approach_instance_2.log
