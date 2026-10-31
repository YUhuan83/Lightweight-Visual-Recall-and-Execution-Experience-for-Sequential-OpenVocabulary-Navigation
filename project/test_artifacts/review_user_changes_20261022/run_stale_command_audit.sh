#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v9/run_full.sh)
for trial in 20261023_102730 20261023_103241; do
    python3 /home/ly/execution_continuity_review_20261022/incoming/audit_stale_command_codex.py \
      "/home/ly/execution_continuity_review_20261022/v10_point_support/full/trial_${trial}/sessions/session_${trial}/goal_001"
done
