#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v2/run_full.sh)
/usr/bin/python3 /home/ly/execution_continuity_review_20261022/incoming/audit_cusp_origin.py
