#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/town_memory_validation_20261014/curvature_contract_candidate1/install/setup.bash
source /home/ly/wildos-console/install/setup.bash
source /home/ly/exploration_owner_repair_20261016/final_install/setup.bash
source /home/ly/frontier_cost_repair_20261016/install/setup.bash
source /home/ly/frontier_cost_repair_20261016/release_install/setup.bash
source /home/ly/exploration_scope_repair_20261016/install/setup.bash
source /home/ly/frontier_resolution_repair_20261016/install/setup.bash
source /home/ly/frontier_motion_handoff_20261016/install/setup.bash
python3 /home/ly/frontier_motion_handoff_20261016/field/analyze_closed.py > /home/ly/frontier_motion_handoff_20261016/closed_analysis.log
python3 /home/ly/frontier_motion_handoff_20261016/review_field.py > /home/ly/frontier_motion_handoff_20261016/closed_summary.log
python3 /home/ly/frontier_motion_handoff_20261016/inspect_closed_progress.py > /home/ly/frontier_motion_handoff_20261016/target_analysis.log
ffprobe -v error -show_entries format=duration:stream=codec_name,nb_frames -of json /home/ly/frontier_motion_handoff_20261016/field_scope_baseline_search_20261016_205441/rviz_live.mp4 > /home/ly/frontier_motion_handoff_20261016/field/video_probe.json
ffmpeg -nostdin -v error -i /home/ly/frontier_motion_handoff_20261016/field_scope_baseline_search_20261016_205441/rviz_live.mp4 -f null - 2> /home/ly/frontier_motion_handoff_20261016/field/video_decode.log
python3 /home/ly/frontier_motion_handoff_20261016/archive_field.py > /home/ly/frontier_motion_handoff_20261016/archive.log
