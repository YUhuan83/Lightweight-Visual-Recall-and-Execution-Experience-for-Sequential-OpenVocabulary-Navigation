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
export ROS_DOMAIN_ID=89
export PYTHONPATH=/home/ly/frontier_motion_handoff_20261016:/home/ly/frontier_motion_handoff_20261016/candidate/nav2_adapter/test:$PYTHONPATH
python3 /home/ly/frontier_motion_handoff_20261016/test_resolution_dds.py > /home/ly/frontier_motion_handoff_20261016/resolution_dds.log 2>&1
tail -n 5 /home/ly/frontier_motion_handoff_20261016/resolution_dds.log
python3 /home/ly/frontier_motion_handoff_20261016/field/verify_before.py
python3 /home/ly/frontier_motion_handoff_20261016/field/drive_sensor_full.py search > /home/ly/frontier_motion_handoff_20261016/driver_launch.log 2>&1
