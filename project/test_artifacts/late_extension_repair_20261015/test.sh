#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/wildos-console/install/setup.bash
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ly/wildos-console/bundle/nav2_adapter/config/fastdds_local_udp.xml
/home/ly/beamng_ros2_0394_ws/.venv/bin/python /home/ly/late_extension_repair_20261015/test_installed.py
/home/ly/beamng_ros2_0394_ws/.venv/bin/python -m unittest discover -s /home/ly/wildos-console/bundle/nav2_adapter/test -p test_graph_path_bridge_routing.py -v
