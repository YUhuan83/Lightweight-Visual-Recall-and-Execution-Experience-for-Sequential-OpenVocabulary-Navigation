#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v48_range_candidate/run_full.sh")
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
export ROS_DOMAIN_ID=213 ROS_LOCALHOST_ONLY=1 PYTHONFAULTHANDLER=1
cd "$base/v48_range_candidate"
cp real_ros_review_1.log real_ros_review_initial_segfault.log
for version in v47_depth_geometry v48_range_candidate; do
 export PYTHONPATH="$base/$version/candidate/wildos_port/repro_check/src/nebula2-wildos/visual_navigation/test:$base/$version/candidate/wildos_port/repro_check/src/nebula2-wildos/visual_navigation:$base/$version/candidate/tools:$base/$version/candidate/nav2_adapter/test:$base/$version/candidate/nav2_adapter:$base/$version/candidate/official_adapter:$PYTHONPATH"
 cd "$base/$version"
 python -X faulthandler real_ros_review.py >real_ros_lifetime_comparison.log 2>&1
 cp real_ros_report.json real_ros_lifetime_comparison.json
 if [[ "$version" == v48_range_candidate ]]; then
  cp real_ros_lifetime_comparison.log real_ros_review_1.log
  cp real_ros_report.json real_ros_report_1.json
  python -X faulthandler real_ros_review.py >real_ros_review_2.log 2>&1
  cp real_ros_report.json real_ros_report_2.json
  for run in 1 2; do "$base/v42_retirement_boundary/build/wildos_ackermann_rpp/test_controller" >"cpp_preserved_$run.log" 2>&1; done
  python candidate/tools/test_wildos_console.py >console_tests.log 2>&1
  python "$base/freeze_range_candidate.py"
 fi
done
