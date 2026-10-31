#!/usr/bin/env bash
set -eo pipefail
python3 /home/ly/planning_phase_physical_20261018/prestart.py
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v26_body_arrival/run_full.sh)
for repeat in 1 2; do
    /home/ly/vision-cu128/bin/python /tmp/v26_lidar_depth_replay.py
    /home/ly/vision-cu128/bin/python /tmp/v26_bearing_replay.py
done
