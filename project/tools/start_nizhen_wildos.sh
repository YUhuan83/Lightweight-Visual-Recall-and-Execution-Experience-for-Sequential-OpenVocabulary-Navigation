#!/usr/bin/env bash
set +u
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
export ROS_DOMAIN_ID=39
export ROS_LOCALHOST_ONLY=1
SRC=/home/ly/wildos-sim/wildos_port/repro_check/src/nebula2-wildos
export PYTHONPATH="$SRC:$PYTHONPATH"
exec /home/ly/vision-cu128/bin/python -m visual_navigation.wildos.nav --config beamng_wildos --do_object_search false
