#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/town_memory_validation_20261014/curvature_contract_candidate1/install/setup.bash
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
colcon --log-base /home/ly/wildos-console/build_logs build --build-base /home/ly/wildos-console/build --install-base /home/ly/wildos-console/install --merge-install --base-paths /home/ly/wildos-console/bundle/nav2_adapter --packages-select wildos_nav2_adapter
source /home/ly/wildos-console/install/setup.bash
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ly/wildos-console/bundle/nav2_adapter/config/fastdds_local_udp.xml
/home/ly/beamng_ros2_0394_ws/.venv/bin/python /home/ly/late_extension_repair_20261015/test_installed.py
