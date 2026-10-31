#!/usr/bin/env bash
set -eo pipefail
python3 /home/ly/planning_phase_physical_20261018/prestart.py
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v28_intent_handoff/run_full.sh)
/home/ly/vision-cu128/bin/python /tmp/audit_v28_mask_identity.py
/home/ly/vision-cu128/bin/python /tmp/v28_lidar_depth_replay.py
