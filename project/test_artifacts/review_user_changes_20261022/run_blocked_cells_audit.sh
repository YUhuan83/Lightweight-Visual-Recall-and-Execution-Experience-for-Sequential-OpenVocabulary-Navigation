#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ly/execution_continuity_review_20261022/v9/install/setup.bash
python3 /home/ly/execution_continuity_review_20261022/incoming/audit_blocked_forward_cells.py > /home/ly/execution_continuity_review_20261022/v9/blocked_cells_repeat_1.json
python3 /home/ly/execution_continuity_review_20261022/incoming/audit_blocked_forward_cells.py > /home/ly/execution_continuity_review_20261022/v9/blocked_cells_repeat_2.json
sha256sum /home/ly/execution_continuity_review_20261022/v9/blocked_cells_repeat_1.json /home/ly/execution_continuity_review_20261022/v9/blocked_cells_repeat_2.json
grep THREE_CELL_CONTROL /home/ly/execution_continuity_review_20261022/v9/early_shunts_probe_1.log /home/ly/execution_continuity_review_20261022/v9/early_shunts_probe_2.log
