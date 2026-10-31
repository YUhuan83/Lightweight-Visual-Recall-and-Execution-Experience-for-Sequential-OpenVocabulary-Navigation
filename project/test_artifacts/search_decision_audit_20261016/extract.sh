#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/wildos-console/install/setup.bash
/home/ly/beamng_ros2_0394_ws/.venv/bin/python /home/ly/search_decision_audit_20261016/extract_decisions.py
