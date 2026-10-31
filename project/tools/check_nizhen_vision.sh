#!/usr/bin/env bash
set +u
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
export ROS_DOMAIN_ID=39
export ROS_LOCALHOST_ONLY=1
SRC=/home/ly/wildos-sim/wildos_port/repro_check/src/nebula2-wildos
export PYTHONPATH="$SRC:$PYTHONPATH"
/home/ly/vision-cu128/bin/python - <<'PY'
import rclpy, torch
import visual_navigation.wildos.nav
print("IMPORT_OK", torch.cuda.is_available(), torch.cuda.device_count())
PY
