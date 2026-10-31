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
source /home/ly/source_control_repair_20261017/install/setup.bash
export ROS_DOMAIN_ID=88
exec /home/ly/beamng_ros2_0394_ws/.venv/bin/python -m wildos_nav2_adapter.dds_profile --profile /home/ly/source_control_repair_20261017/candidate/nav2_adapter/config/fastdds_local_udp.xml -- /home/ly/beamng_ros2_0394_ws/.venv/bin/python /home/ly/source_control_repair_20261017/field/probe_source_lifecycle.py "$1"
