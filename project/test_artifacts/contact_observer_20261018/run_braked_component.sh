#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/native_delay_20261018/run_stack.sh)
source /home/ly/smoothing_boundary_repair_20261018/install/local_setup.bash
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=0 DISPLAY=:148
unset WILDOS_NATIVE_DIAGNOSTIC_BEST_EFFORT
cd /home/ly/contact_observer_20261018
exec /home/ly/beamng_ros2_0394_ws/.venv/bin/python run_braked_component.py
