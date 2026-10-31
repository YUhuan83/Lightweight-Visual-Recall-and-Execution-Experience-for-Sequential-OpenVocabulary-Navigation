#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v9/run_full.sh)
cd /home/ly/execution_continuity_review_20261022/v30_confirmation_history
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
colcon --log-base build_log build --base-paths candidate/nav2_adapter candidate/official_adapter --packages-select wildos_nav2_adapter beamng_wildos_adapter --build-base build --install-base install --parallel-workers 2 >build.log 2>&1
source install/local_setup.bash
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
export ROS_DOMAIN_ID=205 ROS_LOCALHOST_ONLY=1
export PYTHONPATH=$PWD/candidate/wildos_port/repro_check/src/nebula2-wildos/visual_navigation:$PWD/candidate/tools:$PWD/candidate/nav2_adapter/test:$PYTHONPATH
python real_ros_review.py >real_ros_review.log 2>&1
