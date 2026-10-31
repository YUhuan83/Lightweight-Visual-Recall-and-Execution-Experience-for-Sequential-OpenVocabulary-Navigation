#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v51_confirmed_action_handoff/run_full.sh")
for run in 1 2; do
 python3 "$base/audit_local_collision_prefix.py" >"$base/v51_confirmed_action_handoff/local_prefix_audit_$run.log" 2>&1
 cp "$base/v51_confirmed_action_handoff/local_prefix_audit.json" "$base/v51_confirmed_action_handoff/local_prefix_audit_$run.json"
done
cmp "$base/v51_confirmed_action_handoff/local_prefix_audit_1.json" "$base/v51_confirmed_action_handoff/local_prefix_audit_2.json"
cat "$base/v51_confirmed_action_handoff/local_prefix_audit_2.log"
