#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/.venv/bin/activate
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
export ROS_DOMAIN_ID=40 ROS_LOCALHOST_ONLY=1 ROS2CLI_NO_DAEMON=1
python3 /home/ly/navigation_validation_20260930_deploy.py
