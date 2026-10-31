#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/native_delay_20261018/run_stack.sh)
source /home/ly/smoothing_boundary_repair_20261018/install/local_setup.bash
source /home/ly/goal_release_repair_20261019/install/local_setup.bash
export ROS_DOMAIN_ID=195 ROS_LOCALHOST_ONLY=1
cd /home/ly/goal_release_repair_20261019
export PYTHONPATH=$PWD/candidate/tools:$PWD/candidate/nav2_adapter:$PWD/candidate/official_adapter:${PYTHONPATH}
python_bin=/home/ly/beamng_ros2_0394_ws/.venv/bin/python
$python_bin candidate/test_artifacts/goal_release_repair_20261019/run_regression.py --label real_ros --all --real-ros
$python_bin candidate/tools/test_source_control_preflight.py -v
$python_bin candidate/test_artifacts/goal_release_repair_20261019/probe_joint.py
