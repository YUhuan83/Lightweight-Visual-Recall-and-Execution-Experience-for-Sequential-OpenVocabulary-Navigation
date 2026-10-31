#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/planning_phase_physical_20261018/install/setup.bash
export ROS_DOMAIN_ID=183 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ly/planning_phase_physical_20261018/candidate/nav2_adapter/config/fastdds_local_udp.xml
cd /home/ly/native_delay_20261018
python=/home/ly/beamng_ros2_0394_ws/.venv/bin/python
"$python" probe_native_executor.py --external --seconds 25 --output actual_profile.json >actual_profile.log 2>&1
cat actual_profile.log
