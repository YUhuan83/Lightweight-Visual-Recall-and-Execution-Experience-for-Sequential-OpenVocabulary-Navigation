#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v9/run_full.sh)
source /home/ly/execution_continuity_review_20261022/v18_comparison_bound/install/local_setup.bash
exec /home/ly/beamng_ros2_0394_ws/.venv/bin/python /tmp/profile_recorded_dispatch_v18.py
