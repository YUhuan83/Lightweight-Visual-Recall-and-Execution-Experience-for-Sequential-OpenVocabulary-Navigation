#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/native_delay_20261018/run_stack.sh)
source /home/ly/smoothing_boundary_repair_20261018/install/local_setup.bash
export ROS_DOMAIN_ID=88 DISPLAY=:148
export PYTHONPATH=/home/ly/wildos-sim/wildos_port/repro_check/src/nebula2-wildos:$PYTHONPATH
unset WILDOS_NATIVE_DIAGNOSTIC_BEST_EFFORT
export WILDOS_NATIVE_KERNEL_TRACE=1
cd /home/ly/smoothing_boundary_repair_20261018
# Existing frozen driver: --idle keeps command gates closed and sends no query.
# It owns and closes its own process tree; prestart rejects any prior runtime.
exec /home/ly/beamng_ros2_0394_ws/.venv/bin/python \
  /home/ly/smoothing_boundary_repair_20261018/field_native_investigation/driver.py --idle
