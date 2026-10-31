#!/usr/bin/env bash
set +u
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
export ROS_DOMAIN_ID=39
export ROS_LOCALHOST_ONLY=1
for t in /spot1/realsense/front/color/image_raw /spot1/realsense/front/color/camera_info /spot1/odom /spot1/nav_graph /spot1/local_map; do
  echo "== $t =="
  timeout 6 ros2 topic hz "$t" 2>&1 | head -n 3
done
echo "== publishers image =="
ros2 topic info -v /spot1/realsense/front/color/image_raw 2>&1 | grep -E 'Node name|Publisher count|Subscription count'
echo "== publishers odom =="
ros2 topic info -v /spot1/odom 2>&1 | grep -E 'Node name|Publisher count|Subscription count'
