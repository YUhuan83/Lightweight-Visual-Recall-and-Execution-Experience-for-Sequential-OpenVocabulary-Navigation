#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ly/wildos-console/bundle/nav2_adapter/config/fastdds_local_udp.xml
cmake -S /home/ly/sensor_origin_repair_20261015 -B /home/ly/sensor_origin_repair_20261015/build -DCMAKE_BUILD_TYPE=Debug
cmake --build /home/ly/sensor_origin_repair_20261015/build -j2
/home/ly/sensor_origin_repair_20261015/build/observation_origin_probe
