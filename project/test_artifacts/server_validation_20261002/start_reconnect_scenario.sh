#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/.venv/bin/activate
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
export ROS_DOMAIN_ID=40
export ROS_LOCALHOST_ONLY=1
timeout 120 ros2 service call /beamng_bridge/start_scenario beamng_msgs/srv/StartScenario \
  "{path_to_scenario_definition: '/home/ly/full_town_memory_20261002/east_coast_town_reconnect.json'}"
