#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/native_delay_20261018/run_stack.sh)
source /home/ly/vehicle_profile_repair_20261019/install/local_setup.bash
source /home/ly/continuous_goal_repair_20261020/install/local_setup.bash
source /home/ly/frontier_absence_repair_20261020/install/local_setup.bash
export ROS_DOMAIN_ID=88 DISPLAY=:148 CUDA_VISIBLE_DEVICES=0
export PYTHONPATH=/home/ly/exploration_progress_repair_20261020/full/candidate/tools:/home/ly/exploration_progress_repair_20261020/full/candidate/wildos_port/repro_check/src/nebula2-wildos:/home/ly/wildos-sim/wildos_port/repro_check/src/nebula2-wildos:${PYTHONPATH}
unset WILDOS_NATIVE_DIAGNOSTIC_BEST_EFFORT
cd /home/ly/exploration_progress_repair_20261020
/home/ly/beamng_ros2_0394_ws/.venv/bin/python prepare_full.py
exec /home/ly/beamng_ros2_0394_ws/.venv/bin/python -m wildos_nav2_adapter.dds_profile --profile full/candidate/nav2_adapter/config/fastdds_local_udp.xml -- /home/ly/beamng_ros2_0394_ws/.venv/bin/python full/driver_full.py
