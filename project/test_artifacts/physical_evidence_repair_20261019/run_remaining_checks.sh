#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/native_delay_20261018/run_stack.sh)
source /home/ly/smoothing_boundary_repair_20261018/install/local_setup.bash
root=/home/ly/physical_evidence_repair_20261019
source "$root/candidate6/install/local_setup.bash"
cd "$root/candidate6"
export ROS_LOCALHOST_ONLY=1
export PYTHONPATH=$PWD/candidate/tools:$PWD/candidate/nav2_adapter:$PWD/candidate/official_adapter:$PWD/candidate/wildos_port/repro_check/src/nebula2-wildos/visual_navigation:${PYTHONPATH}
python_bin=/home/ly/beamng_ros2_0394_ws/.venv/bin/python
export ROS_DOMAIN_ID=197
$python_bin candidate/test_artifacts/recall_contract_repair_20261019/run_tests.py test_execution_prior_contract test_directional_coverage_contract test_visual_memory test_visual_memory_map test_scoring_execution_contract test_target_bearing test_object_memory_integration test_recall_component_names > recall_tests.log 2>&1
$python_bin candidate/test_artifacts/passability_contract_20261019/run_ground_tests.py > ground_tests.log 2>&1
$python_bin candidate/test_artifacts/goal_release_repair_20261019/run_regression.py --label real_ros --all --real-ros > real_ros.log 2>&1
export ROS_DOMAIN_ID=195
$python_bin candidate/test_artifacts/goal_release_fence_20261019/probe_joint.py > goal_release_dds.log 2>&1
export ROS_DOMAIN_ID=196
$python_bin "$root/probe_recall_dds.py" > recall_dds.log 2>&1
export ROS_DOMAIN_ID=199
$python_bin "$root/replay_candidate.py" > recorded_replay.log 2>&1
