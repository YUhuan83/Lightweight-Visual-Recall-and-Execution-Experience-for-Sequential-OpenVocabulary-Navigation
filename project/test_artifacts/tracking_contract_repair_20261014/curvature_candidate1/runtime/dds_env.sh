#!/usr/bin/env bash
set -e
TASK=/home/ly/town_memory_validation_20261014/curvature_contract_candidate1
PARENT=/home/ly/town_memory_validation_20261014/tracking_contract_candidate
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source "$PARENT/install/setup.bash"
source "$TASK/install/setup.bash"
export ROS_DOMAIN_ID=88
export PYTHONPATH="$TASK/bundle/tools:$TASK/install/lib/python3.10/site-packages:$PARENT/install/lib/python3.10/site-packages:/home/ly/wildos-sim/wildos_port/repro_check/src/nebula2-wildos:${PYTHONPATH}"
exec /home/ly/beamng_ros2_0394_ws/.venv/bin/python -m wildos_nav2_adapter.dds_profile -- "$@"
