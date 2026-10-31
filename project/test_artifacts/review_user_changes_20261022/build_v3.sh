#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v2/run_full.sh)
source /home/ly/execution_continuity_review_20261022/v3/install/local_setup.bash
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
cd /home/ly/execution_continuity_review_20261022/v3
colcon --log-base build_log build --base-paths candidate/nav2_adapter candidate/official_adapter candidate/ackermann_smac --packages-select wildos_nav2_adapter beamng_wildos_adapter wildos_ackermann_smac --build-base build --install-base install --parallel-workers 2 >build.log 2>&1
source install/local_setup.bash
colcon test --build-base build --install-base install --packages-select wildos_ackermann_smac --event-handlers console_direct+ >smac_tests.log 2>&1
colcon test-result --test-result-base build/wildos_ackermann_smac --verbose >>smac_tests.log 2>&1
export ROS_DOMAIN_ID=204 ROS_LOCALHOST_ONLY=1
export PYTHONPATH=$PWD/candidate/nav2_adapter/test:$PYTHONPATH
python real_ros_review.py >real_ros_review.log 2>&1
