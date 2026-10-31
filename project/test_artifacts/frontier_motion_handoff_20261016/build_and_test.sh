#!/usr/bin/env bash
set -eo pipefail
export PATH="$HOME/.local/bin:$PATH"
REMOTE="/home/ly/frontier_motion_handoff_20261016"
mkdir -p "$REMOTE/candidate"
cp "$REMOTE/manifest.json" "$REMOTE/official_wildos_beamng_bundle.manifest.json"
python3 - "$REMOTE" <<'PY'
from pathlib import Path
import sys, zipfile, hashlib, json
r=Path(sys.argv[1]); m=json.loads((r/'manifest.json').read_text())
assert hashlib.sha256((r/'candidate.zip').read_bytes()).hexdigest()==m['sha256']
with zipfile.ZipFile(r/'candidate.zip') as z:
    assert set(z.namelist())==set(m['members'])
    for n,h in m['members'].items():
        assert hashlib.sha256(z.read(n)).hexdigest()==h,n
    z.extractall(r/'candidate')
print('candidate verified',m['sha256'],len(m['members']))
PY
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
source /home/ly/town_memory_validation_20261014/curvature_contract_candidate1/install/setup.bash
source /home/ly/wildos-console/install/setup.bash
source /home/ly/exploration_owner_repair_20261016/final_install/setup.bash
source /home/ly/frontier_cost_repair_20261016/install/setup.bash
source /home/ly/frontier_cost_repair_20261016/release_install/setup.bash
source /home/ly/exploration_scope_repair_20261016/install/setup.bash
source /home/ly/frontier_resolution_repair_20261016/install/setup.bash

cd "$REMOTE"
/home/ly/beamng_ros2_0394_ws/.venv/bin/colcon --log-base "$REMOTE/build_logs" build --base-paths "$REMOTE/candidate/nav2_adapter" --packages-select wildos_nav2_adapter --build-base "$REMOTE/build" --install-base "$REMOTE/install" --merge-install > "$REMOTE/build.log" 2>&1
source "$REMOTE/install/setup.bash"
export ROS_DOMAIN_ID=89
export PYTHONPATH="$REMOTE/candidate/nav2_adapter/test:$REMOTE/candidate/official_adapter:$PYTHONPATH"
python3 -m unittest test_graph_path_bridge_routing.ExplorationTargetHandoffTests test_exploration_cost_ros -v > "$REMOTE/real_ros_callbacks_final.log" 2>&1
python3 - > "$REMOTE/ownership_callbacks.log" 2>&1 <<'PY'
import rclpy,unittest
rclpy.init()
result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromName('test_planner_execution_ownership_ros.PlannerOwnershipTests'))
rclpy.shutdown()
raise SystemExit(0 if result.wasSuccessful() else 1)
PY
python3 - "$REMOTE" <<'PY'
from pathlib import Path
import importlib, hashlib, json,sys
r=Path(sys.argv[1]); report={}
for name in ('graph_path_bridge','continuous_path','execution_feedback'):
    p=Path(importlib.import_module('wildos_nav2_adapter.'+name).__file__)
    h=hashlib.sha256(p.read_bytes()).hexdigest()
    assert h==hashlib.sha256((r/'candidate/nav2_adapter/wildos_nav2_adapter'/p.name).read_bytes()).hexdigest()
    report[name]={'path':str(p),'sha256':h}
(r/'installed_identity.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
PY
tail -n 5 "$REMOTE/real_ros_callbacks_final.log"
tail -n 5 "$REMOTE/ownership_callbacks.log"
