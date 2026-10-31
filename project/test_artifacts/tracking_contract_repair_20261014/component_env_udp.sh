#!/usr/bin/env bash
set -e
TASK=/home/ly/town_memory_validation_20261014/tracking_contract_candidate
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source "$TASK/install/setup.bash"
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE="$TASK/component_tools/fastdds_loopback_udp.xml"
export PYTHONPATH="$TASK/install/lib/python3.10/site-packages:/home/ly/wildos-sim/wildos_port/repro_check/src/nebula2-wildos:${PYTHONPATH}"
exec "$@"
