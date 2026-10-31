#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v9/run_full.sh)
source /home/ly/execution_continuity_review_20261022/v19_dispatch_reuse/install/local_setup.bash
exec /home/ly/beamng_ros2_0394_ws/.venv/bin/python /tmp/profile_recorded_dispatch_v19.py
