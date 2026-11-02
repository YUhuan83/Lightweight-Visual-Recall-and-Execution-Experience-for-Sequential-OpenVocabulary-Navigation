#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd -- "$(dirname -- "$0")/.." && pwd)
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH LD_PRELOAD
source /opt/ros/humble/setup.bash
cd "$ROOT"
chmod +x "$ROOT/src/graphnav_mapper/scripts/dummy_pose_graph_publisher.py"
"${COLCON_EXECUTABLE:-colcon}" build --base-paths src --symlink-install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3
test -x "$ROOT/install/graphnav_mapper/lib/graphnav_mapper/dummy_pose_graph_publisher.py"
