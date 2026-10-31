#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/wildos-console/install/setup.bash
/home/ly/beamng_ros2_0394_ws/.venv/bin/python /home/ly/sensor_origin_repair_20261015/analyze_field_cloud.py > /home/ly/sensor_origin_repair_20261015/field_cloud_origin_review.json
/home/ly/beamng_ros2_0394_ws/.venv/bin/python /home/ly/sensor_origin_repair_20261015/analyze_field_closed.py > /home/ly/sensor_origin_repair_20261015/field_closed_analysis.json
