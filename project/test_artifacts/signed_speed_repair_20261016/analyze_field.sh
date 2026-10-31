#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/wildos-console/install/setup.bash
/home/ly/beamng_ros2_0394_ws/.venv/bin/python /home/ly/signed_speed_repair_20261016/analyze_field_cloud.py > /home/ly/signed_speed_repair_20261016/field_cloud_origin_review.json
python3 /home/ly/signed_speed_repair_20261016/spatial_review.py > /home/ly/signed_speed_repair_20261016/spatial_review.json
/home/ly/beamng_ros2_0394_ws/.venv/bin/python /home/ly/signed_speed_repair_20261016/analyze_full_paths.py > /home/ly/signed_speed_repair_20261016/full_path_handoff_review.json
python3 /home/ly/signed_speed_repair_20261016/analyze_signed_speed.py > /home/ly/signed_speed_repair_20261016/signed_speed_review.json
python3 /home/ly/signed_speed_repair_20261016/analyze_state_alignment.py > /home/ly/signed_speed_repair_20261016/state_clock_alignment_review.json
/home/ly/beamng_ros2_0394_ws/.venv/bin/python /home/ly/signed_speed_repair_20261016/analyze_field_closed.py > /home/ly/signed_speed_repair_20261016/field_closed_analysis.json

