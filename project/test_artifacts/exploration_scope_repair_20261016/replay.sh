#!/usr/bin/env bash
set -eo pipefail
source /home/ly/exploration_efficiency_audit_20261016/env.sh
source /home/ly/frontier_cost_repair_20261016/install/setup.bash
source /home/ly/exploration_scope_repair_20261016/install/setup.bash
export PYTHONPATH=/home/ly/exploration_owner_repair_20261016:$PYTHONPATH
audit_root=/home/ly/exploration_scope_repair_20261016
/home/ly/beamng_ros2_0394_ws/.venv/bin/python "$audit_root/test_scope_dds.py" >"$audit_root/scope_dds.log" 2>&1
/home/ly/beamng_ros2_0394_ws/.venv/bin/python "$audit_root/validate_replay.py" >"$audit_root/replay_summary.log" 2>&1
