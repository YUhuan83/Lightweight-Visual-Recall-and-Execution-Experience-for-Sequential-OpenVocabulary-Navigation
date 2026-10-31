#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/planning_phase_physical_20261018/install/setup.bash
export ROS_DOMAIN_ID=181
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ly/planning_phase_physical_20261018/candidate/nav2_adapter/config/fastdds_local_udp.xml
export RMW_FASTRTPS_USE_QOS_FROM_XML=0
cd /home/ly/execution_wait_audit_20261018/startup_receipt
python=/home/ly/beamng_ros2_0394_ws/.venv/bin/python
for mode in reliable1 reliable10 best1; do
  args=(--depth 1)
  if [ "$mode" = reliable10 ]; then args=(--depth 10); fi
  if [ "$mode" = best1 ]; then args+=(--best-effort); fi
  "$python" qos_probe.py reader "${args[@]}" --seconds 17 --output "$mode.json" >"$mode.log" 2>&1 &
  reader=$!
  "$python" qos_probe.py publisher --seconds 16 >"${mode}_publisher.log" 2>&1 &
  publisher=$!
  wait "$publisher"
  wait "$reader"
  cat "$mode.log"
done
