#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v37_full_path/run_full.sh")
cd "$base/v42_retirement_boundary"
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
colcon --log-base build_log build --base-paths candidate/nav2_adapter candidate/official_adapter candidate/ackermann_rpp --packages-select wildos_nav2_adapter beamng_wildos_adapter wildos_ackermann_rpp --build-base build --install-base install --parallel-workers 2 >build.log 2>&1
export ROS_DOMAIN_ID=212 ROS_LOCALHOST_ONLY=1
# New test executable, unchanged preserved v37 plugin registry/library.
for run in 1 2; do
  set +e
  build/wildos_ackermann_rpp/test_controller --gtest_filter='ActualPlugin.ContinuousRetirement*' >"cpp_before_$run.log" 2>&1
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
for run in 1 2; do
  python real_ros_review.py >"real_ros_review_$run.log" 2>&1
  cp real_ros_report.json "real_ros_report_$run.json"
done
python candidate/tools/test_wildos_console.py >console_tests.log 2>&1
python "$base/freeze_retirement_boundary.py"
