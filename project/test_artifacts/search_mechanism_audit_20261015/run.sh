#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/wildos-console/install/setup.bash
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ly/wildos-console/bundle/nav2_adapter/config/fastdds_local_udp.xml
/usr/bin/python3 /home/ly/search_mechanism_audit_20261015/extract_inputs.py
cmake -S /home/ly/search_mechanism_audit_20261015 -B /home/ly/search_mechanism_audit_20261015/build -DCMAKE_BUILD_TYPE=Release
cmake --build /home/ly/search_mechanism_audit_20261015/build -j4
/home/ly/search_mechanism_audit_20261015/build/connector_probe /home/ly/search_mechanism_audit_20261015/input0.txt
/home/ly/search_mechanism_audit_20261015/build/connector_probe /home/ly/search_mechanism_audit_20261015/input1.txt
