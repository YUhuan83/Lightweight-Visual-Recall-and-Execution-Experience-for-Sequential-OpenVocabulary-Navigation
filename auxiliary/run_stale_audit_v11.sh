#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v9/run_full.sh)
python3 /home/ly/execution_continuity_review_20261022/incoming/audit_stale_command_codex.py \
  /home/ly/execution_continuity_review_20261022/v11_path_recovery/full/trial_20261023_172602/sessions/session_20261023_172602/goal_001
