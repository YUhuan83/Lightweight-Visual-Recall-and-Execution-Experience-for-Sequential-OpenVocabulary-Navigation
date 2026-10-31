#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v9/run_full.sh)
source "$1/install/local_setup.bash"
exec /home/ly/beamng_ros2_0394_ws/.venv/bin/python /tmp/profile_stop_plan_replay.py "$1"
