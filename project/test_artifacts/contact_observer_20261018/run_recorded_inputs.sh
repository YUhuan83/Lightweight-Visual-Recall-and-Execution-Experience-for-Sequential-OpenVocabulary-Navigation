#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/native_delay_20261018/run_stack.sh)
source /home/ly/smoothing_boundary_repair_20261018/install/local_setup.bash
cd /home/ly/recorded_section_replay_20261018
exec /home/ly/beamng_ros2_0394_ws/.venv/bin/python test_recorded_ros_inputs.py
