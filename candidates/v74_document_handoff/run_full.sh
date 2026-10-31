#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/confirmed_approach_recovery_20261020/v2/run_full.sh)
source /home/ly/execution_continuity_review_20261022/v1/install/local_setup.bash
source /home/ly/execution_continuity_review_20261022/v2/install/local_setup.bash
source /home/ly/execution_continuity_review_20261022/v3/install/local_setup.bash
source /home/ly/execution_continuity_review_20261022/v4/install/local_setup.bash
source /home/ly/execution_continuity_review_20261022/v5/install/local_setup.bash
source /home/ly/execution_continuity_review_20261022/v6/install/local_setup.bash
source /home/ly/execution_continuity_review_20261022/v7/install/local_setup.bash
source /home/ly/execution_continuity_review_20261022/v8/install/local_setup.bash
source /home/ly/execution_continuity_review_20261022/v9/install/local_setup.bash
source /home/ly/execution_continuity_review_20261022/v26_body_arrival/install/local_setup.bash
source /home/ly/execution_continuity_review_20261022/v42_retirement_boundary/install/local_setup.bash
source /home/ly/execution_continuity_review_20261022/v42_retirement_boundary/install/local_setup.bash
source /home/ly/execution_continuity_review_20261022/v42_retirement_boundary/install/local_setup.bash
source /home/ly/execution_continuity_review_20261022/v42_retirement_boundary/install/local_setup.bash
source /home/ly/execution_continuity_review_20261022/v42_retirement_boundary/install/local_setup.bash
source /home/ly/execution_continuity_review_20261022/v55_frontier_endpoint/install/local_setup.bash
source /home/ly/execution_continuity_review_20261022/v66_original_approach_120s/install/local_setup.bash
source /home/ly/execution_continuity_review_20261022/v66_original_approach_120s/install/local_setup.bash
source /home/ly/execution_continuity_review_20261022/v67_target_road_120s/install/local_setup.bash
source /home/ly/execution_continuity_review_20261022/v70_upstream_arrival_120s/install/local_setup.bash
source /home/ly/execution_continuity_review_20261022/v71_heading_freshness/install/local_setup.bash
source /home/ly/execution_continuity_review_20261022/v73_document_plan/install/local_setup.bash
source /home/ly/execution_continuity_review_20261022/v74_document_handoff/install/local_setup.bash
export PYTHONPATH=/home/ly/execution_continuity_review_20261022/v74_document_handoff/candidate/wildos_port/repro_check/src/nebula2-wildos/visual_navigation:/home/ly/execution_continuity_review_20261022/v74_document_handoff/full/candidate/tools:/home/ly/execution_continuity_review_20261022/v74_document_handoff/full/candidate/wildos_port/repro_check/src/nebula2-wildos:${PYTHONPATH}
cd /home/ly/execution_continuity_review_20261022/v74_document_handoff
exec /home/ly/beamng_ros2_0394_ws/.venv/bin/python -m wildos_nav2_adapter.dds_profile --profile full/candidate/nav2_adapter/config/fastdds_local_udp.xml -- /home/ly/beamng_ros2_0394_ws/.venv/bin/python full/driver_arc_capture.py
