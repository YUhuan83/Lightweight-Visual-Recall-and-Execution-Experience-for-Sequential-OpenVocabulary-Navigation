#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/native_delay_20261018/run_stack.sh)
source /home/ly/smoothing_boundary_repair_20261018/install/local_setup.bash
exec /home/ly/beamng_ros2_0394_ws/.venv/bin/python /home/ly/passability_contract_20261019/extract_inputs.py
