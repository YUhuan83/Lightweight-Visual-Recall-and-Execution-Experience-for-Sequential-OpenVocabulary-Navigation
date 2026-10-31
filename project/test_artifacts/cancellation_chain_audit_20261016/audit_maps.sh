#!/usr/bin/env bash
set -eo pipefail
source /home/ly/exploration_efficiency_audit_20261016/env.sh
source /home/ly/exploration_scope_repair_20261016/install/setup.bash
source /home/ly/frontier_resolution_repair_20261016/install/setup.bash
python3 /home/ly/cancellation_chain_audit_20261016/audit_maps.py
