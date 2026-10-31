#!/usr/bin/env bash
set +u
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
export ROS_DOMAIN_ID=39
export ROS_LOCALHOST_ONLY=1
SCEN=/home/ly/beamng_ros2_0394_ws/src/beamng-ros2-integration/beamng_ros2/config/scenarios/wildos_official_window.json
echo "== reset scenario =="
timeout 60 ros2 service call /beamng_bridge/start_scenario beamng_msgs/srv/StartScenario \
  "{path_to_scenario_definition: '$SCEN'}" 2>&1 | tail -n 3
sleep 8
echo "== odom after reset =="
timeout 6 ros2 topic echo /spot1/odom --field pose.pose.position --once 2>/dev/null
