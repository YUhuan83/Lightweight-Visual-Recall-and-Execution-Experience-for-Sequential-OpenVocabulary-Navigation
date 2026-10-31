#!/usr/bin/env bash
set +u
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
export ROS_DOMAIN_ID=39
export ROS_LOCALHOST_ONLY=1
echo "== /control sample =="
timeout 6 ros2 topic echo /control --once 2>/dev/null
echo "== /wildos/cmd_vel publishers =="
ros2 topic info -v /wildos/cmd_vel 2>&1 | grep -E 'Publisher count|Node name|Subscription count'
echo "== node list (control chain) =="
ros2 node list 2>/dev/null | grep -iE 'controller|planner|cmd_vel|nav2|bridge' || true
echo "== bridge control apply log =="
grep -n -iE 'control|apply|throttle|gear' /tmp/official_bridge.log 2>/dev/null | tail -n 10
echo "== electrics =="
timeout 6 ros2 topic echo /vehicles/ego/sensors/electrics --once 2>/dev/null | grep -iE 'throttle|brake|gear|parking|rpm|wheelspeed|running' | head -n 20
