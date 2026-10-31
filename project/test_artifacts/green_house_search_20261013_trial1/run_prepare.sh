#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/town_memory_validation_20261013/navigation_transactions_candidate/install/setup.bash
export ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1
export PYTHONPATH=/home/ly/town_memory_validation_20261013/navigation_transactions_candidate/install/lib/python3.10/site-packages:${PYTHONPATH}
exec /home/ly/beamng_ros2_0394_ws/.venv/bin/python /home/ly/town_memory_validation_20261013/prepare_round.py --output /home/ly/town_memory_validation_20261013/green_house_trial1
