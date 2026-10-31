#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/wildos-console/install/setup.bash
python3 /home/ly/full_score_search_20261015/prepare_replay.py
/usr/bin/python3 /home/ly/full_score_search_20261015/extract_inputs.py > /home/ly/full_score_search_20261015/replay_inputs.log
for i in 0 1 2; do
 echo "case=$i"
 /home/ly/search_mechanism_audit_20261015/build/connector_probe /home/ly/full_score_search_20261015/input$i.txt
done
