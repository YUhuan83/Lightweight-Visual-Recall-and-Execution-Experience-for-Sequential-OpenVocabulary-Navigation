#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/native_delay_20261018/run_stack.sh)
export ROS_DOMAIN_ID=88 DISPLAY=:148
export PYTHONPATH=/home/ly/wildos-sim/wildos_port/repro_check/src/nebula2-wildos:$PYTHONPATH
export WILDOS_NATIVE_DIAGNOSTIC_BEST_EFFORT=1
exec /home/ly/beamng_ros2_0394_ws/.venv/bin/python drive_dynamic.py --idle
