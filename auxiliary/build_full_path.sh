#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v35_confirmed_vehicle/run_full.sh)
cd /home/ly/execution_continuity_review_20261022/v37_full_path
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
colcon --log-base build_log build --base-paths candidate/nav2_adapter candidate/official_adapter candidate/ackermann_rpp --packages-select wildos_nav2_adapter beamng_wildos_adapter wildos_ackermann_rpp --build-base build --install-base install --parallel-workers 2 >build.log 2>&1
export ROS_DOMAIN_ID=207 ROS_LOCALHOST_ONLY=1
# Exactly the new cases, using the preserved OLD controller/plugin registry.
# Both old runs must fail before the candidate overlay is enabled.
for run in 1 2; do
  set +e
  build/wildos_ackermann_rpp/test_controller --gtest_filter='ActualPlugin.Continuous*' >"cpp_before_$run.log" 2>&1
  result=$?
  set -e
  test "$result" -ne 0
done
source install/local_setup.bash
for run in 1 2; do
  build/wildos_ackermann_rpp/test_controller >"cpp_after_$run.log" 2>&1
  build/wildos_ackermann_rpp/test_curvature >"curvature_$run.log" 2>&1
done
export PYTHONPATH=$PWD/candidate/wildos_port/repro_check/src/nebula2-wildos/visual_navigation:$PWD/candidate/tools:$PWD/candidate/nav2_adapter/test:$PYTHONPATH
python real_ros_review.py >real_ros_review.log 2>&1
printf 'BUILD_AND_COMPONENTS_PASSED\n'
