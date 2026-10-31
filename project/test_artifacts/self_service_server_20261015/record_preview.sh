#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/wildos-console/install/setup.bash
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ly/wildos-console/bundle/nav2_adapter/config/fastdds_local_udp.xml
/home/ly/beamng_ros2_0394_ws/.venv/bin/python /home/ly/record_preview_20261015.py
