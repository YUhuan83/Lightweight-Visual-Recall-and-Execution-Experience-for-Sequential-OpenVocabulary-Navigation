#!/usr/bin/env bash
set -eo pipefail
round_root=$1
source <(sed '/^exec /,$d' "$round_root/run_full.sh")
/home/ly/beamng_ros2_0394_ws/.venv/bin/python /tmp/audit_arrival_round.py "$round_root"
