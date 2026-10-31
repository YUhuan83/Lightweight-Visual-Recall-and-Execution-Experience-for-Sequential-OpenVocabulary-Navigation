#!/usr/bin/env bash
set -eo pipefail
R=/home/ly/exploration_owner_field_20261016
for trial_wait in $(seq 1 300); do
 if [ -f /home/ly/wildos-console/field_exploration_owner_search_20261016_095739/result.json ] && [ -f /home/ly/wildos-console/audit_results/session_20261016_095740/session_result.json ]; then break; fi
 sleep 2
done
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/town_memory_validation_20261014/curvature_contract_candidate1/install/setup.bash
source /home/ly/wildos-console/install/setup.bash
source /home/ly/exploration_owner_repair_20261016/final_install/setup.bash
/home/ly/beamng_ros2_0394_ws/.venv/bin/python "$R/analyze_closed.py" > "$R/closed_review_stdout.log"
ffprobe -v error -show_streams -show_format -of json /home/ly/wildos-console/field_exploration_owner_search_20261016_095739/rviz_live.mp4 > "$R/video_probe.json"
python3 "$R/pack_closed.py"
