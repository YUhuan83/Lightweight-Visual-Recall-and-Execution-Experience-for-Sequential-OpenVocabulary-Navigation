#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v54_confirmed_graph_handoff/run_full.sh")
for run in 1 2; do
 python3 "$base/audit_v54_instance_projection.py" >"$base/v54_confirmed_graph_handoff/instance_projection_$run.log" 2>&1
 cp "$base/v54_confirmed_graph_handoff/instance_projection_audit/report.json" "$base/v54_confirmed_graph_handoff/instance_projection_audit/report_$run.json"
done
cmp "$base/v54_confirmed_graph_handoff/instance_projection_audit/report_1.json" "$base/v54_confirmed_graph_handoff/instance_projection_audit/report_2.json"
cat "$base/v54_confirmed_graph_handoff/instance_projection_2.log"
