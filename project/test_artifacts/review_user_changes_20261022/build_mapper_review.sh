#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v1/run_full.sh)
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
cd /home/ly/execution_continuity_review_20261022/v2
export ROS_DOMAIN_ID=205 ROS_LOCALHOST_ONLY=1
colcon --log-base build_log build --base-paths candidate/wildos_port/repro_check/src/nebula2-wildos/graphnav_mapper --packages-select graphnav_mapper --build-base build --install-base install --parallel-workers 1 --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON >mapper_build.log 2>&1
source install/local_setup.bash
build/graphnav_mapper/graphnav_mapper_test_mapper --gtest_filter='MapperTest.FreshlyBlockedOldProxy*:MapperTest.BlockedProxyCan*:MapperTest.StaleBlockedProxy*' >mapper_counterexamples_before.log 2>&1
