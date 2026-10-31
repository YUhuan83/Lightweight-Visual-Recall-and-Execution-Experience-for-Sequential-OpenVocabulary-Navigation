#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/confirmed_approach_recovery_20261020/v2/run_full.sh)
source /home/ly/execution_continuity_repair_20261021/v4/install/local_setup.bash
cd /home/ly/execution_continuity_repair_20261021/v4
unset DIAGNOSTIC_BEST_EFFORT
export ROS_DOMAIN_ID=183 ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/candidate/nav2_adapter/config/fastdds_local_udp.xml
export LD_PRELOAD=/home/ly/execution_continuity_repair_20261021/transport_probe/drop_one.so
export WILDOS_DROP_LOG=$PWD/native_loss_1_drops.jsonl
/home/ly/beamng_ros2_0394_ws/.venv/bin/python probe_native_loss.py --external --seconds 7 --output native_loss_1.json >native_loss_1.log 2>&1
export WILDOS_DROP_LOG=$PWD/native_loss_2_drops.jsonl
/home/ly/beamng_ros2_0394_ws/.venv/bin/python probe_native_loss.py --external --seconds 7 --output native_loss_2.json >native_loss_2.log 2>&1
export WILDOS_DROP_LOG=$PWD/native_loss_3_drops.jsonl
/home/ly/beamng_ros2_0394_ws/.venv/bin/python probe_native_loss.py --external --seconds 7 --output native_loss_3.json >native_loss_3.log 2>&1
