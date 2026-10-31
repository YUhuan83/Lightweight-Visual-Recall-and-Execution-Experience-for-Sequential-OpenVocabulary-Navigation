#!/usr/bin/env bash
set +u
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
export ROS_DOMAIN_ID=39
export ROS_LOCALHOST_ONLY=1

echo "== odom before =="
timeout 5 ros2 topic echo /spot1/odom --field pose.pose.position --once 2>/dev/null

echo "== electrics before =="
timeout 5 ros2 topic echo /vehicles/ego/sensors/electrics --once 2>/dev/null | grep -iE 'running|gear|throttle|brake|parking|rpm|wheelspeed|fuel|engine' | head -n 20

echo "== drive forward for 8s =="
timeout 8 ros2 topic pub --rate 20 /control beamng_msgs/msg/VehicleControl \
  "{steering: 0.0, throttle: 0.35, brake: 0.0, parkingbrake: 0.0, clutch: 0.0, gear: 2}" > /tmp/drive_test.log 2>&1
echo "drive test done"

echo "== odom after =="
timeout 5 ros2 topic echo /spot1/odom --field pose.pose.position --once 2>/dev/null

echo "== electrics after =="
timeout 5 ros2 topic echo /vehicles/ego/sensors/electrics --once 2>/dev/null | grep -iE 'running|gear|throttle|brake|parking|rpm|wheelspeed|engine' | head -n 20
