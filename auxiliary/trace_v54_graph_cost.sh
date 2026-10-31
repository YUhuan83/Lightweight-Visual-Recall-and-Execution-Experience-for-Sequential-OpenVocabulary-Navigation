#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v54_confirmed_graph_handoff/run_full.sh")
root="$base/frontier_endpoint_contract_review"
python3 "$base/trace_v54_graph_cost.py"
cmake -S "$root" -B "$root/build" >"$root/trace_configure.log" 2>&1
cmake --build "$root/build" --target replay_trace -j2 >"$root/trace_build.log" 2>&1
TRACE_PLAN=1 "$root/build/replay_trace" "$root/trace_graph.txt" geometry_only >"$root/trace_geometry.log" 2>&1
TRACE_PLAN=1 "$root/build/replay_trace" "$root/trace_graph.txt" >"$root/trace_scored.log" 2>&1
grep '^TRACE' "$root/trace_geometry.log"
