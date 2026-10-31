#!/usr/bin/env bash
set +u
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
export ROS_DOMAIN_ID=39
export ROS_LOCALHOST_ONLY=1
timeout 8 ros2 service call /beamng_bridge/pause beamng_msgs/srv/ChangeSimulationState "{}"
timeout 8 ros2 topic echo /vehicles/ego/sensors/electrics beamng_msgs/msg/ElectricsSensor --once > /tmp/post_stop_electrics.txt
grep -E '^brake:|^brake_input:|^throttle:|^throttle_input:|^wheelspeed:' /tmp/post_stop_electrics.txt
