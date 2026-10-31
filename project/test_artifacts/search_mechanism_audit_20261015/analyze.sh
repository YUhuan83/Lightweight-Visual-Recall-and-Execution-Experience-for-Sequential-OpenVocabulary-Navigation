#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/wildos-console/install/setup.bash
/usr/bin/python3 /home/ly/search_mechanism_audit_20261015/analyze_and_archive.py
