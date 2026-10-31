#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/wildos-console/install/setup.bash
exec /home/ly/beamng_ros2_0394_ws/.venv/bin/python /home/ly/sensor_origin_repair_20261015/replay_recorded_origins.py
