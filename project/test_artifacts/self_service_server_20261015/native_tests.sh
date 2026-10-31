#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/town_memory_validation_20261014/curvature_contract_candidate1/install/setup.bash
source /home/ly/wildos-console/install/setup.bash
export ROS_DOMAIN_ID=89
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ly/wildos-console/bundle/nav2_adapter/config/fastdds_local_udp.xml
ctest --test-dir /home/ly/wildos-console/build/wildos_ackermann_smac --output-on-failure
ctest --test-dir /home/ly/wildos-console/build/wildos_ackermann_rpp --output-on-failure
