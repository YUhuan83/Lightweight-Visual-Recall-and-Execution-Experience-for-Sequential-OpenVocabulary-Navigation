#!/usr/bin/env bash
set -eo pipefail
base=/home/ly/execution_continuity_review_20261022
source <(sed '/^exec /,$d' "$base/v42_retirement_boundary/run_full.sh")
cd "$base/v54_confirmed_graph_handoff"
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
colcon --log-base build_log build --base-paths candidate/nav2_adapter candidate/official_adapter --packages-select wildos_nav2_adapter beamng_wildos_adapter --build-base build --install-base install --parallel-workers 2 >build.log 2>&1
source install/local_setup.bash
export ROS_DOMAIN_ID=213 ROS_LOCALHOST_ONLY=1
export PYTHONPATH=$PWD/candidate/wildos_port/repro_check/src/nebula2-wildos/visual_navigation/test:$PWD/candidate/wildos_port/repro_check/src/nebula2-wildos/visual_navigation:$PWD/candidate/tools:$PWD/candidate/nav2_adapter/test:$PYTHONPATH
for run in 1 2; do
  python real_ros_review.py >"real_ros_review_$run.log" 2>&1
  cp real_ros_report.json "real_ros_report_$run.json"
  "$base/v42_retirement_boundary/build/wildos_ackermann_rpp/test_controller" >"cpp_preserved_$run.log" 2>&1
done
python candidate/tools/test_physical_sensor_rig.py >rig_test.log 2>&1
python candidate/tools/test_wildos_console.py >console_tests.log 2>&1
python "$base/freeze_confirmed_graph_handoff.py"
