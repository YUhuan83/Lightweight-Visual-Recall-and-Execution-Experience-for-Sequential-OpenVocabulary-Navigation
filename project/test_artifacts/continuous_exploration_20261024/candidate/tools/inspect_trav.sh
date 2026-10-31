#!/usr/bin/env bash
set +u
M=$(find /home/ly/beamng_ros2_0394_ws/src /home/ly/wildos-sim -maxdepth 6 -type d -name graphnav_mapper 2>/dev/null | head -n1)
echo "MAPPER=$M"
if [ -n "$M" ]; then
  grep -rn -E "trav|Trav|footprint|clearance|vehicle_radius|robot" "$M" 2>/dev/null | grep -v -E "\.pyc|/build/|/install/" | head -n 50
fi
echo "=== traversability class config ==="
grep -rn -E "traversability_class|trav_properties|trav_class" /home/ly/beamng_ros2_0394_ws/src 2>/dev/null | grep -v -E "\.pyc|/build/|/install/" | head -n 30
