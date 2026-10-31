#!/usr/bin/env bash
set +u
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash 2>/dev/null || true

for pkg in graphnav_msgs graphnav_mapper graphnav_planner visual_navigation; do
  echo "=== $pkg ==="
  ros2 pkg prefix "$pkg" 2>/dev/null || echo MISSING
  find /home/ly/beamng_ros2_0394_ws/src /home/ly/wildos-sim -maxdepth 8 -type d -name "$pkg" 2>/dev/null | head -n 4
done

echo "=== dependencies ==="
ros2 pkg prefix grid_map_msgs 2>/dev/null || echo GRID_MAP_MSGS_MISSING
ros2 pkg prefix graaf_vendor 2>/dev/null || echo GRAAF_VENDOR_MISSING
df -h /home/ly | tail -n 1
