#!/usr/bin/env bash
# Build one frozen arm into its own ROS overlay. Never starts simulation.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")" && pwd)
python3 "$ROOT/verify_experiment.py" --syntax-only
test -f /opt/ros/humble/setup.bash || { echo 'ROS 2 Humble is required on the Linux test host'; exit 1; }
set +u
source /opt/ros/humble/setup.bash
set -u
command -v colcon >/dev/null
cd "$ROOT"
colcon --log-base "$ROOT/log" build --base-paths \
  "$ROOT/nav2_adapter" "$ROOT/official_adapter" \
  "$ROOT/wildos_port/repro_check/src/nebula2-wildos" \
  --build-base "$ROOT/build" --install-base "$ROOT/install" --symlink-install \
  --event-handlers console_direct+
echo "Built isolated arm at $ROOT/install; no motion started"
