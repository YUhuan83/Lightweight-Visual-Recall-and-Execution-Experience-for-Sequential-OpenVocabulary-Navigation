#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/wildos-console/install/setup.bash
/home/ly/beamng_ros2_0394_ws/.venv/bin/python /home/ly/planner_wait_audit_20261015/analyze_field_cloud.py > /home/ly/planner_wait_audit_20261015/field_cloud_origin_review.json
/home/ly/beamng_ros2_0394_ws/.venv/bin/python /home/ly/planner_wait_audit_20261015/analyze_field_closed.py > /home/ly/planner_wait_audit_20261015/field_closed_analysis.json

python3 /home/ly/planner_wait_audit_20261015/spatial_review.py > /home/ly/planner_wait_audit_20261015/spatial_review.json
