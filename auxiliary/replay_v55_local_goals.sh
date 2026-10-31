#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/$1/run_full.sh")
for run in 1 2; do
 python3 "$base/replay_v55_local_goals.py" "$1" >"$base/$1/local_goal_replay_$run.log" 2>&1
 cp "$base/$1/local_goal_replay.json" "$base/$1/local_goal_replay_$run.json"
done
cmp "$base/$1/local_goal_replay_1.json" "$base/$1/local_goal_replay_2.json"
cat "$base/$1/local_goal_replay_2.log"
