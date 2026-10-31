#!/usr/bin/env bash
set +u
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
export ROS_DOMAIN_ID=39
export ROS_LOCALHOST_ONLY=1
pkill -9 -f 'mjpeg_stream.py' 2>/dev/null || true
sleep 1
exec /home/ly/vision-cu128/bin/python /home/ly/mjpeg_stream.py --host 127.0.0.1 --port 8080
