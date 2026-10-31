#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v9/run_full.sh)
source /home/ly/execution_continuity_review_20261022/v17_costmap_unblock_retry/install/local_setup.bash
exec /home/ly/beamng_ros2_0394_ws/.venv/bin/python /tmp/replay_v16_blocked_start_v17.py
