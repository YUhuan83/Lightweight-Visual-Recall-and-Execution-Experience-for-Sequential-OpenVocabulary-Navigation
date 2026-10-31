#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/town_memory_validation_20261014/curvature_contract_candidate1/install/setup.bash
source /home/ly/wildos-console/install/setup.bash
export DISPLAY=:148 ROS_DOMAIN_ID=88
export PYTHONPATH=/home/ly/wildos-console/bundle/tools:/home/ly/wildos-sim/wildos_port/repro_check/src/nebula2-wildos:$PYTHONPATH
exec /home/ly/beamng_ros2_0394_ws/.venv/bin/python -m wildos_nav2_adapter.dds_profile --profile /home/ly/wildos-console/bundle/nav2_adapter/config/fastdds_local_udp.xml -- /home/ly/beamng_ros2_0394_ws/.venv/bin/python /home/ly/search_mechanism_audit_20261015/audit_console.py --runtime --config /home/ly/wildos-console/audit_console.json
