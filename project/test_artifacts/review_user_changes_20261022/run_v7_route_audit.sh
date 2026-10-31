#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v7/run_full.sh)
python3 /home/ly/execution_continuity_review_20261022/incoming/audit_v7_route_recovery.py
