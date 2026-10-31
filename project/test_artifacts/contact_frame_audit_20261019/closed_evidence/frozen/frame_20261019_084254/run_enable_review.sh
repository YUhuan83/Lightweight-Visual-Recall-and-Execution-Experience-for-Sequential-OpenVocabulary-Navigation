#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/native_delay_20261018/run_stack.sh)
source /home/ly/smoothing_boundary_repair_20261018/install/local_setup.bash
cd /home/ly/contact_frame_audit_20261019
exec /home/ly/beamng_ros2_0394_ws/.venv/bin/python inspect_enable_boundary.py /home/ly/contact_frame_audit_20261019/frame_20261019_083934
