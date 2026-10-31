#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/planning_phase_physical_20261018/install/setup.bash
export ROS_DOMAIN_ID=183 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ly/planning_phase_physical_20261018/candidate/nav2_adapter/config/fastdds_local_udp.xml
export PYTHONPATH=/home/ly/planning_phase_physical_20261018/candidate/tools:$PYTHONPATH
cd /home/ly/native_delay_20261018
exec /home/ly/beamng_ros2_0394_ws/.venv/bin/python physical_brake_probe.py
