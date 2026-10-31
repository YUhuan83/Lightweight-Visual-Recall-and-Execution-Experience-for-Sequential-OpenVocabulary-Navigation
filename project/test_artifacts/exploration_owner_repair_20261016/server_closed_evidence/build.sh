#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/town_memory_validation_20261014/curvature_contract_candidate1/install/setup.bash
source /home/ly/wildos-console/install/setup.bash
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
R=/home/ly/exploration_owner_repair_20261016
colcon --log-base "$R/build_logs" build --build-base "$R/build" --install-base "$R/install" --merge-install --base-paths "$R/candidate/wildos_port/repro_check/src/nebula2-wildos/graphnav_planner" --packages-select graphnav_planner --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
source "$R/install/setup.bash"
cd "$R/build/graphnav_planner"
ctest --output-on-failure -R '^test_route_mechanism$'
