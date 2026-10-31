#!/usr/bin/env bash
set -eo pipefail
source /home/ly/exploration_efficiency_audit_20261016/env.sh
source /home/ly/exploration_scope_repair_20261016/install/setup.bash
export PYTHONPATH=/home/ly/exploration_owner_repair_20261016:$PYTHONPATH
python3 /home/ly/exploration_scope_repair_20261016/test_resolution_dds_before.py > /home/ly/exploration_scope_repair_20261016/resolution_before.log 2>&1
