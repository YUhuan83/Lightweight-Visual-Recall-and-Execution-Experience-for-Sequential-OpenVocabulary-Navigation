#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/planning_phase_physical_20261018/install/setup.bash
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ly/planning_phase_physical_20261018/candidate/nav2_adapter/config/fastdds_local_udp.xml
cd /home/ly/execution_wait_audit_20261018/startup_receipt
python=/home/ly/beamng_ros2_0394_ws/.venv/bin/python
"$python" /home/ly/planning_phase_physical_20261018/prestart.py
stamp=$(date +%Y%m%d_%H%M%S)
"$python" shadow_readers.py "shadow_${stamp}.jsonl" >"shadow_${stamp}.log" 2>&1 &
observer=$!
cleanup() { kill -TERM "$observer" 2>/dev/null || true; wait "$observer" || true; }
trap cleanup EXIT
bash /home/ly/planning_phase_physical_20261018/run_field.sh >"field_${stamp}.log" 2>&1
