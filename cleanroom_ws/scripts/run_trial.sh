#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd -- "$(dirname -- "$0")/.." && pwd)
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH LD_PRELOAD
source /opt/ros/humble/setup.bash
source "$ROOT/install/local_setup.bash"
export WILDOS_CONFIG_DIR="$ROOT/config"
export PYTHONPATH="$ROOT/vendor:$ROOT/vendor/nvidia_radio:${PYTHONPATH}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
export PYTHONUNBUFFERED=1
exec "${CONTROL_PYTHON:-python3}" "$ROOT/scripts/run_trial.py" "$@"
