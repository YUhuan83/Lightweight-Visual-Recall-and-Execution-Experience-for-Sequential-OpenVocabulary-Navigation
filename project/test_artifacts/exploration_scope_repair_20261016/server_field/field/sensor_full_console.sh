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
export DISPLAY=:148 ROS_DOMAIN_ID=88
export PYTHONPATH=/home/ly/exploration_scope_repair_20261016/candidate/tools:/home/ly/wildos-sim/wildos_port/repro_check/src/nebula2-wildos:$PYTHONPATH
exec /home/ly/beamng_ros2_0394_ws/.venv/bin/python -m wildos_nav2_adapter.dds_profile --profile /home/ly/exploration_scope_repair_20261016/candidate/nav2_adapter/config/fastdds_local_udp.xml -- /home/ly/beamng_ros2_0394_ws/.venv/bin/python /home/ly/exploration_scope_repair_20261016/field/sensor_console.py --runtime --config /home/ly/exploration_scope_repair_20261016/field/sensor_full_console.json
