#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v9/run_full.sh)
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=0
python3 -m wildos_nav2_adapter.dds_profile --profile /home/ly/execution_continuity_review_20261022/v9/candidate/nav2_adapter/config/fastdds_local_udp.xml -- python3 /home/ly/execution_continuity_review_20261022/incoming/observe_parameter_services.py
