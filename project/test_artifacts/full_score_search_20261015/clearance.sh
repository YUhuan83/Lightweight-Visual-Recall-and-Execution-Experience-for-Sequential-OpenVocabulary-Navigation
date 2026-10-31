#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/wildos-console/install/setup.bash
cmake -S /home/ly/full_score_search_20261015 -B /home/ly/full_score_search_20261015/build -DCMAKE_BUILD_TYPE=Release
cmake --build /home/ly/full_score_search_20261015/build -j2
for i in 0 1 2; do
 echo "case=$i"
 /home/ly/full_score_search_20261015/build/probe_clearance /home/ly/full_score_search_20261015/input$i.txt
done
