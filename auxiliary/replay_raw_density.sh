#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v37_full_path/run_full.sh)
export PYTHONPATH=/home/ly/execution_continuity_review_20261022/v37_full_path/candidate/wildos_port/repro_check/src/nebula2-wildos/visual_navigation:$PYTHONPATH
cd /home/ly/execution_continuity_review_20261022
for run in 1 2; do
  python3 replay_raw_density.py >"v37_full_path/raw_density_replay_$run.log" 2>&1
  cp v37_full_path/raw_density_replay.json "v37_full_path/raw_density_replay_$run.json"
done
cat v37_full_path/raw_density_replay_2.log
