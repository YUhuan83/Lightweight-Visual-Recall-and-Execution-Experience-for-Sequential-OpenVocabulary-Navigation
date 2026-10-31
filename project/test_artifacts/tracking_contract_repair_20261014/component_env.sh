#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/town_memory_validation_20261014/tracking_contract_candidate/install/setup.bash
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1
export PYTHONPATH=/home/ly/town_memory_validation_20261014/tracking_contract_candidate/install/lib/python3.10/site-packages:/home/ly/wildos-sim/wildos_port/repro_check/src/nebula2-wildos:${PYTHONPATH}
exec "$@"
