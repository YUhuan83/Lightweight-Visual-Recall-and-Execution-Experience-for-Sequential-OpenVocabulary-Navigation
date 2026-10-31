#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/native_delay_20261018/run_stack.sh)
source /home/ly/smoothing_boundary_repair_20261018/install/local_setup.bash
cd /home/ly/goal_release_repair_20261019
/home/ly/beamng_ros2_0394_ws/.venv/bin/colcon --log-base build_logs build --base-paths candidate/wildos_port/repro_check/src/nebula2-wildos/graphnav_planner --build-base build --install-base install --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/local_setup.bash
export ROS_DOMAIN_ID=195 ROS_LOCALHOST_ONLY=1
./build/graphnav_planner/test_route_mechanism --gtest_output=xml:cpp_results.xml
