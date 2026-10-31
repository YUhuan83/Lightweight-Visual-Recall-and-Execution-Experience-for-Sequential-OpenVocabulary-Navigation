#!/usr/bin/env bash
set -eo pipefail
# Reuse the exact source overlay order, without its final executable line.
source <(sed '/^exec /,$d' /home/ly/native_delay_20261018/run_stack.sh)
export ROS_DOMAIN_ID=88
export PYTHONPATH=/home/ly/wildos-sim/wildos_port/repro_check/src/nebula2-wildos:$PYTHONPATH
exec /home/ly/beamng_ros2_0394_ws/.venv/bin/python physical_brake_probe.py --stack --vision
