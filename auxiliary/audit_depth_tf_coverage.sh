#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v25_fast_shift/run_full.sh)
export ROS_DOMAIN_ID=89
for repeat in 1 2; do
    /home/ly/vision-cu128/bin/python /tmp/audit_depth_tf_coverage.py
done
