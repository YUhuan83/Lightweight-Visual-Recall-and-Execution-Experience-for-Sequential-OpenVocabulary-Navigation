#!/usr/bin/env bash
set +u
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
export ROS_DOMAIN_ID=39
export ROS_LOCALHOST_ONLY=1
export DISPLAY=:3
export XAUTHORITY=/run/user/1011/gdm/Xauthority
pkill -9 -f 'rviz2' 2>/dev/null || true
pkill -9 -f 'crashReporter' 2>/dev/null || true
pkill -9 -f 'xfce4-screensaver' 2>/dev/null || true
sleep 2
xset s off 2>/dev/null || true
xset -dpms 2>/dev/null || true
exec rviz2 -d /home/ly/nizhen_wildos.rviz
