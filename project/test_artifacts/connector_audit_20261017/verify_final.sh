#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/wildos-console/install/setup.bash
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
export ROS_DOMAIN_ID=175
cd /home/ly/connector_audit_20261017
source regression_install_candidate/setup.bash
colcon --log-base regression_test_final_log test --base-paths regression_sources/candidate/ackermann_smac --build-base regression_build_candidate --install-base regression_install_candidate --event-handlers console_direct+ >regression_candidate_final.log 2>&1
colcon test-result --test-result-base regression_build_candidate --verbose
regression_build_candidate/wildos_ackermann_smac/test_planner --gtest_filter=ActualPlanner.RecordedCompleteRouteAvoidsFirstFeasibleFourCuspDetour --gtest_repeat=3 >regression_repeat.log 2>&1
mkdir -p results_final
cd results_final
ln -sf ../input*.txt .
sha256sum ../regression_install_candidate/wildos_ackermann_smac/lib/libwildos_ackermann_smac.so >library_identity.txt
ldd ../build/recorded_planner >linked_libraries.txt
for index in $(seq 0 18); do
 GEOMETRY_CASE=$index GEOMETRY_SMOOTH=1 ../build/recorded_planner --gtest_filter=ActualPlanner.FullTrialRecordedGeometryAudit >case${index}.log 2>&1
 grep GEOMETRY_PLAN case${index}.log
done
