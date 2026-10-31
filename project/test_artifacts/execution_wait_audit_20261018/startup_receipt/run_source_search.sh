#!/usr/bin/env bash
set -eo pipefail
root=/home/ly/planning_phase_physical_20261018
diag=/home/ly/execution_wait_audit_20261018/startup_receipt
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/town_memory_validation_20261014/curvature_contract_candidate1/install/setup.bash
source /home/ly/wildos-console/install/setup.bash
source /home/ly/exploration_owner_repair_20261016/final_install/setup.bash
source /home/ly/frontier_cost_repair_20261016/install/setup.bash
source /home/ly/frontier_cost_repair_20261016/release_install/setup.bash
source /home/ly/exploration_scope_repair_20261016/install/setup.bash
source /home/ly/frontier_resolution_repair_20261016/install/setup.bash
source /home/ly/frontier_motion_handoff_20261016/install/setup.bash
source /home/ly/visible_pursuit_repair_20261017/install/setup.bash
source /home/ly/full_vehicle_acceptance_20261017/install/setup.bash
source /home/ly/motion_handoff_field_20261018/install/setup.bash
source /home/ly/controller_source_contract_20261018/install/setup.bash
source /home/ly/planning_phase_physical_20261018/install/setup.bash
export DISPLAY=:148
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$root/candidate/nav2_adapter/config/fastdds_local_udp.xml"
python=/home/ly/beamng_ros2_0394_ws/.venv/bin/python
"$python" "$root/prestart.py"
"$python" "$root/field/verify_before.py"
stamp=$(date +%Y%m%d_%H%M%S)
"$python" "$diag/shadow_search_readers.py" "$diag/search_shadow_${stamp}.jsonl" 620 >"$diag/search_shadow_${stamp}.log" 2>&1 &
observer=$!
cleanup() { kill -TERM "$observer" 2>/dev/null || true; wait "$observer" || true; }
trap cleanup EXIT
"$python" "$root/field/drive_sensor_full.py" search >"$diag/search_field_${stamp}.log" 2>&1
