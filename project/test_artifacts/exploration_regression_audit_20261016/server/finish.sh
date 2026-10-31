#!/usr/bin/env bash
set -eo pipefail
source /home/ly/exploration_efficiency_audit_20261016/env.sh
source /home/ly/frontier_cost_repair_20261016/install/setup.bash
source /home/ly/frontier_cost_repair_20261016/release_install/setup.bash
audit_root=/home/ly/exploration_regression_audit_20261016
cmake --build "$audit_root/build" -j2 >"$audit_root/extended_build.log" 2>&1
/home/ly/beamng_ros2_0394_ws/.venv/bin/python "$audit_root/finish_audit.py"
