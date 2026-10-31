#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v25_fast_shift/run_full.sh)
for repeat in 1 2; do
    /home/ly/beamng_ros2_0394_ws/.venv/bin/python /tmp/audit_visible_arc_options.py
done
