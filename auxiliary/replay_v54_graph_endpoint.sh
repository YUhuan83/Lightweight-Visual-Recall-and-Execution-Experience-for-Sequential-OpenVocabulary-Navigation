#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v54_confirmed_graph_handoff/run_full.sh")
root="$base/frontier_endpoint_contract_review"
python3 "$base/prepare_v54_graph_replay.py"
cmake -S "$root" -B "$root/build" >"$root/replay_configure.log" 2>&1
cmake --build "$root/build" -j2 >"$root/replay_build.log" 2>&1
for variant in before after; do
 for run in 1 2; do
  "$root/build/replay_$variant" "$root/recorded_graphs.txt" >"$root/replay_${variant}_${run}.log" 2>&1
 done
 diff <(grep '^RESULT' "$root/replay_${variant}_1.log") <(grep '^RESULT' "$root/replay_${variant}_2.log")
done
echo 'Both recorded-graph replays completed twice.'
