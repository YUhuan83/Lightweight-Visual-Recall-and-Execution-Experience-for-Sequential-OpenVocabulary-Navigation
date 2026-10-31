#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/planning_phase_physical_20261018/install/setup.bash
export ROS_DOMAIN_ID=181 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ly/planning_phase_physical_20261018/candidate/nav2_adapter/config/fastdds_local_udp.xml
cd /home/ly/execution_wait_audit_20261018/startup_receipt
python=/home/ly/beamng_ros2_0394_ws/.venv/bin/python
for mode in reliable1 reliable10 best1; do
  args=(--depth 1)
  if [ "$mode" = reliable10 ]; then args=(--depth 10); fi
  if [ "$mode" = best1 ]; then args+=(--best-effort); fi
  "$python" qos_stall_probe.py reader "${args[@]}" --stall-sec .2 --seconds 17 --output "${mode}_stall.json" >"${mode}_stall.log" 2>&1 &
  reader=$!
  "$python" qos_stall_probe.py publisher --seconds 16 >"${mode}_stall_publisher.log" 2>&1 &
  publisher=$!
  wait "$publisher"
  wait "$reader"
  cat "${mode}_stall.log"
done
