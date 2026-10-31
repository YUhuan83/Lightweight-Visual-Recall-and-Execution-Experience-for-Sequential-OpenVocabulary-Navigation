#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
root=$base/v41_general_observation
source <(sed '/^exec /,$d' "$root/run_full.sh")
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
export ROS_DOMAIN_ID=210 ROS_LOCALHOST_ONLY=1
export PYTHONPATH=$root/candidate/wildos_port/repro_check/src/nebula2-wildos/visual_navigation:$root/candidate/tools:$root/candidate/nav2_adapter/test:$PYTHONPATH
cd "$root"
test ! -e real_ros_initial_failure.log
cp real_ros_review.log real_ros_initial_failure.log
cp real_ros_report.json real_ros_initial_failure_report.json
for repeat in 1 2; do
  python real_ros_review.py >"real_ros_review_$repeat.log" 2>&1
  cp real_ros_report.json "real_ros_report_$repeat.json"
done
cp real_ros_review_2.log real_ros_review.log
for repeat in 1 2; do
  "$base/v37_full_path/build/wildos_ackermann_rpp/test_controller" >"cpp_preserved_$repeat.log" 2>&1
done
python candidate/tools/test_wildos_console.py >console_tests.log 2>&1
python "$base/freeze_general_observation.py"
python "$base/run_confirmed_progress_trial.py" v41_general_observation
