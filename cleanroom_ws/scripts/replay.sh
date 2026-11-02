#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd -- "$(dirname -- "$0")/.." && pwd)
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH LD_PRELOAD
source /opt/ros/humble/setup.bash
source "$ROOT/install/local_setup.bash"
cd "$ROOT"
MODE=$1
shift
case "$MODE" in
  graph) exec build/graphnav_planner/replay_graph "$@" ;;
  smac) exec build/wildos_ackermann_smac/replay_smac "$@" ;;
  debug-smac) exec gdb --batch -ex run -ex bt --args build/wildos_ackermann_smac/replay_smac "$@" ;;
  cloud) exec "${TEST_PYTHON:-python3}" tests/replay_cloud.py "$@" ;;
  *) echo 'Usage: replay.sh graph|smac|cloud [data arguments]' >&2; exit 2 ;;
esac
