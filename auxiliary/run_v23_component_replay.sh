#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v9/run_full.sh)
source /home/ly/execution_continuity_review_20261022/v23_arc_heading/install/local_setup.bash
export ROS_DOMAIN_ID=209 ROS_LOCALHOST_ONLY=1
export PYTHONPATH=/home/ly/execution_continuity_review_20261022/v23_arc_heading/candidate/nav2_adapter/test:$PYTHONPATH
/home/ly/beamng_ros2_0394_ws/.venv/bin/python -m unittest -q test_graph_route
/home/ly/beamng_ros2_0394_ws/.venv/bin/python /tmp/replay_arc_bridge_contract.py
