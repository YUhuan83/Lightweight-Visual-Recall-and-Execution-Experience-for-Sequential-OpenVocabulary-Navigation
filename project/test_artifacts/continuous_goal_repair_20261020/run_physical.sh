#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/native_delay_20261018/run_stack.sh)
source /home/ly/vehicle_profile_repair_20261019/install/local_setup.bash
source /home/ly/continuous_goal_repair_20261020/install/local_setup.bash
export ROS_LOCALHOST_ONLY=1
exec /home/ly/venvs/beamng-039/bin/python /home/ly/continuous_goal_repair_20261020/physical/run_short_component.py
