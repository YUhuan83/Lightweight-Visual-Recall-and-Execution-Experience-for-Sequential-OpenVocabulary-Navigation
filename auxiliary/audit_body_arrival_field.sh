#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v26_body_arrival/run_full.sh)
for repeat in 1 2; do
    /home/ly/beamng_ros2_0394_ws/.venv/bin/python /tmp/audit_body_arrival_field.py
done
