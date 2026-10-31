#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v57_upstream_target_approach/run_full.sh")
python3 "$base/audit_upstream_target_handoff.py"
