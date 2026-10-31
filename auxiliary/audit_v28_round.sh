#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v28_intent_handoff/run_full.sh)
/home/ly/beamng_ros2_0394_ws/.venv/bin/python /tmp/audit_arrival_round.py /home/ly/execution_continuity_review_20261022/v28_intent_handoff
