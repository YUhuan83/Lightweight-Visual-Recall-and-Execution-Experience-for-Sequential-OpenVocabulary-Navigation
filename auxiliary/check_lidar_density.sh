#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v37_full_path/run_full.sh)
python3 /home/ly/execution_continuity_review_20261022/check_lidar_density.py
