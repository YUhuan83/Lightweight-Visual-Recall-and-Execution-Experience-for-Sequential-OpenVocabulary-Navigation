#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v43_abort_recovery/run_full.sh)
export PYTHONPATH=/home/ly/execution_continuity_review_20261022/v43_abort_recovery/candidate/wildos_port/repro_check/src/nebula2-wildos/visual_navigation:$PYTHONPATH
cd /home/ly/execution_continuity_review_20261022
for run in 1 2; do
  python3 replay_binding_isolated.py >"v43_abort_recovery/binding_depth_isolated_$run.log" 2>&1
  cp v43_abort_recovery/binding_depth_isolated.json "v43_abort_recovery/binding_depth_isolated_$run.json"
done
cat v43_abort_recovery/binding_depth_isolated_2.log

cmp v43_abort_recovery/binding_depth_isolated_1.json v43_abort_recovery/binding_depth_isolated_2.json
