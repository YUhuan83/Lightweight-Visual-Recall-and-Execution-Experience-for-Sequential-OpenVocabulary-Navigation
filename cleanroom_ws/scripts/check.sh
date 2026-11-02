#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd -- "$(dirname -- "$0")/.." && pwd)
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH LD_PRELOAD
source /opt/ros/humble/setup.bash
source "$ROOT/install/local_setup.bash"
cd "$ROOT"
bash scripts/test.sh
"${COLCON_EXECUTABLE:-colcon}" test --base-paths src \
  --packages-select graphnav_planner wildos_ackermann_smac wildos_ackermann_rpp \
  --executor sequential --return-code-on-test-failure
"${COLCON_EXECUTABLE:-colcon}" test-result --verbose
