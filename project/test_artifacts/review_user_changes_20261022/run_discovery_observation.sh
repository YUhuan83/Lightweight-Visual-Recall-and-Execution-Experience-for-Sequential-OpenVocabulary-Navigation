#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_review_20261022/v9/run_full.sh)
python3 -m wildos_nav2_adapter.dds_profile --profile /home/ly/execution_continuity_review_20261022/v9/candidate/nav2_adapter/config/fastdds_local_udp.xml -- python3 /home/ly/execution_continuity_review_20261022/incoming/observe_discovery.py
