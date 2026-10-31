#!/usr/bin/env bash
set -eo pipefail
TASK=/home/ly/town_memory_validation_20261014/tracking_contract_candidate
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source "$TASK/install/setup.bash"
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1
export PYTHONPATH="$TASK/resume_session1/source/tools:$TASK/install/lib/python3.10/site-packages:/home/ly/wildos-sim/wildos_port/repro_check/src/nebula2-wildos:${PYTHONPATH}"
exec /home/ly/beamng_ros2_0394_ws/.venv/bin/python "$TASK/resume_session1/source/tools/check_ackermann_cusp_execution.py" --output "$TASK/physical_cusp_component3" --arm "$TASK/resume_session1/source"
