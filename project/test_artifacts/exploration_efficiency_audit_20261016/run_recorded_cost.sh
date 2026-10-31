#!/usr/bin/env bash
set -eo pipefail
source /home/ly/exploration_efficiency_audit_20261016/env.sh
source /home/ly/frontier_cost_repair_20261016/install/setup.bash
source /home/ly/frontier_cost_repair_20261016/release_install/setup.bash
export PYTHONPATH=/home/ly/frontier_cost_repair_20261016/release_candidate/nav2_adapter/test:$PYTHONPATH
/home/ly/beamng_ros2_0394_ws/.venv/bin/python /home/ly/frontier_cost_repair_20261016/recorded_cost_bridge.py
