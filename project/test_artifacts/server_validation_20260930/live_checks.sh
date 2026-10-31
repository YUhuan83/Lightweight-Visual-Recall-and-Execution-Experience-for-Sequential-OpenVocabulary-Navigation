#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/.venv/bin/activate
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
export ROS_DOMAIN_ID=40 ROS_LOCALHOST_ONLY=1 ROS2CLI_NO_DAEMON=1
python3 /home/ly/navigation_validation_20260930_live_probe.py
python3 /home/ly/navigation_validation_20260930_stage/tools/official_planner_probe.py --ahead 4.5 10 --timeout 10 | tee /home/ly/navigation_validation_20260930_logs/planner_probe.json
