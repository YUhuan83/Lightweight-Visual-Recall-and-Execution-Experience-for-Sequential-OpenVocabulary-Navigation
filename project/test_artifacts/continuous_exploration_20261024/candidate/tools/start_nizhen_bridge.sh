#!/usr/bin/env bash
set +u
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/.venv/bin/activate
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
export ROS_DOMAIN_ID=39
export ROS_LOCALHOST_ONLY=1

ros2 service call /beamng_bridge/start_scenario beamng_msgs/srv/StartScenario \
  "{path_to_scenario_definition: '/home/ly/beamng_ros2_0394_ws/src/beamng-ros2-integration/beamng_ros2/config/scenarios/wildos_official_window.json'}"
sleep 10
ros2 topic list | grep -E 'vehicles/ego/sensors|/tf' || true
