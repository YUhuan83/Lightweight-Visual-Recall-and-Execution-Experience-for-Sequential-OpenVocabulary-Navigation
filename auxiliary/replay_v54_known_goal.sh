#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v54_confirmed_graph_handoff/run_full.sh")
root="$base/frontier_endpoint_contract_review"
cmake --build "$root/build" -j2 >"$root/known_goal_build.log" 2>&1
for run in 1 2; do
 "$root/build/replay_after" "$root/recorded_graphs.txt" geometry_only >"$root/known_goal_$run.log" 2>&1
done
diff <(grep '^RESULT' "$root/known_goal_1.log") <(grep '^RESULT' "$root/known_goal_2.log")
echo 'Known-position cost ablation completed twice; no runtime change.'
