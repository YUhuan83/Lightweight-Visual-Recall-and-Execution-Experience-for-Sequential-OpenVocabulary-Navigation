#!/usr/bin/env bash
set -eo pipefail
source /home/ly/exploration_efficiency_audit_20261016/env.sh
source /home/ly/frontier_cost_repair_20261016/install/setup.bash
source /home/ly/frontier_cost_repair_20261016/release_install/setup.bash
R=/home/ly/frontier_cost_repair_20261016
/home/ly/beamng_ros2_0394_ws/.venv/bin/python "$R/field/analyze_closed.py" > "$R/field/closed_analysis.log"
/home/ly/beamng_ros2_0394_ws/.venv/bin/python "$R/analyze_cost_closed.py" > "$R/field/cost_analysis.log"
/home/ly/beamng_ros2_0394_ws/.venv/bin/python "$R/field/pack_closed.py" > "$R/field/pack.log"
