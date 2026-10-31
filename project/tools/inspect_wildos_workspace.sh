#!/usr/bin/env bash
set +u
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash 2>/dev/null || true

echo "=== workspace src ==="
find /home/ly/beamng_ros2_0394_ws/src -maxdepth 3 -mindepth 1 -printf '%y %p -> %l\n' 2>/dev/null | head -n 100

echo "=== colcon package paths ==="
cd /home/ly/beamng_ros2_0394_ws || exit 1
colcon list | grep -E '^(graphnav_msgs|graphnav_mapper|graphnav_planner|visual_navigation)[[:space:]]'

echo "=== build source dirs ==="
for pkg in graphnav_msgs graphnav_mapper graphnav_planner visual_navigation; do
  grep -m1 '^CMAKE_HOME_DIRECTORY:' "/home/ly/beamng_ros2_0394_ws/build/$pkg/CMakeCache.txt" 2>/dev/null || true
done
