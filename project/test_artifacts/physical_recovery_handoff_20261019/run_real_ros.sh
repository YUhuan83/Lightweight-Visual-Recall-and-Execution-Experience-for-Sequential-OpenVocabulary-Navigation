#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/native_delay_20261018/run_stack.sh)
source /home/ly/smoothing_boundary_repair_20261018/install/local_setup.bash
export ROS_DOMAIN_ID=193 ROS_LOCALHOST_ONLY=1
cd /home/ly/physical_recovery_handoff_20261019/candidate
exec /home/ly/beamng_ros2_0394_ws/.venv/bin/python test_artifacts/physical_recovery_handoff_20261019/run_regression.py --label real_ros --all --real-ros
