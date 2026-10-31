#!/usr/bin/env bash
set -eo pipefail
export PATH="$HOME/.local/bin:$PATH"
REMOTE="/home/ly/source_control_repair_20261017"
python3 - "$REMOTE" <<'PY'
import hashlib,json,sys,zipfile,shutil,os
from pathlib import Path
r=Path(sys.argv[1])
if (r/'candidate_next.zip').exists():
 backup=r/'initial_a69';backup.mkdir(exist_ok=True)
 for name in ('candidate.zip','manifest.json','real_ros_callbacks.log','pure_linux.log','build_and_test.sh'):
  if (r/name).is_file() and not (backup/name).exists():shutil.copy2(r/name,backup/name)
 next_manifest=json.loads((r/'manifest_next.json').read_text())
 assert hashlib.sha256((r/'candidate_next.zip').read_bytes()).hexdigest()==next_manifest['sha256']
 os.replace(r/'candidate_next.zip',r/'candidate.zip');os.replace(r/'manifest_next.json',r/'manifest.json')
m=json.loads((r/'manifest.json').read_text())
assert hashlib.sha256((r/'candidate.zip').read_bytes()).hexdigest()==m['sha256']
for name,subdir in [('candidate.zip','candidate'),('harness.zip','')]:
 with zipfile.ZipFile(r/name) as z:
  assert z.testzip() is None
  assert all(not Path(n).is_absolute() and '..' not in Path(n).parts for n in z.namelist())
  if subdir:
   assert set(z.namelist())==set(m['members'])
   for n,h in m['members'].items():assert hashlib.sha256(z.read(n)).hexdigest()==h,n
  z.extractall(r/subdir)
(r/'official_wildos_beamng_bundle.manifest.json').write_text(json.dumps(m,indent=2))
print('verified',m['sha256'],len(m['members']))
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
source /home/ly/frontier_motion_handoff_20261016/install/setup.bash
cd "$REMOTE"
/home/ly/beamng_ros2_0394_ws/.venv/bin/colcon --log-base "$REMOTE/build_logs" build --base-paths "$REMOTE/candidate/nav2_adapter" --packages-select wildos_nav2_adapter --build-base "$REMOTE/build" --install-base "$REMOTE/install" --merge-install > "$REMOTE/build.log" 2>&1
source "$REMOTE/install/setup.bash"
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1
export PYTHONPATH="$REMOTE/candidate/nav2_adapter/test:$REMOTE/candidate/tools:$REMOTE/candidate/official_adapter:$PYTHONPATH"
python3 -m unittest test_source_control test_source_control_preflight test_dds_profile test_runtime_contract -v > "$REMOTE/pure_linux.log" 2>&1
python3 - > "$REMOTE/real_ros_callbacks.log" 2>&1 <<'PY'
import rclpy,unittest
rclpy.init()
suite=unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromName(n) for n in
 ('test_source_control_ros','test_native_vehicle_control_ros','test_planner_execution_ownership_ros.PlannerOwnershipTests'))
result=unittest.TextTestRunner(verbosity=2).run(suite)
rclpy.shutdown()
raise SystemExit(not result.wasSuccessful())
PY
python3 -m unittest test_graph_path_bridge_routing.ExplorationTargetHandoffTests test_exploration_cost_ros -v > "$REMOTE/handoff_cost_callbacks.log" 2>&1
python3 "$REMOTE/candidate/tools/check_physical_launch.py" --output "$REMOTE/launch_expansion.json" > "$REMOTE/launch_expansion.log" 2>&1
/home/ly/beamng_ros2_0394_ws/.venv/bin/python - "$REMOTE" <<'PY'
import importlib,hashlib,json,sys
from pathlib import Path
r=Path(sys.argv[1]);m=json.loads((r/'manifest.json').read_text());report={}
for name in ('source_control','cmd_vel_to_control','native_vehicle_control','execution_feedback','graph_path_bridge','runtime_contract'):
 p=Path(importlib.import_module('wildos_nav2_adapter.'+name).__file__)
 h=hashlib.sha256(p.read_bytes()).hexdigest();assert h==m['members']['nav2_adapter/wildos_nav2_adapter/'+p.name],p
 report[name]=dict(path=str(p),sha256=h)
(r/'installed_identity.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
PY
tail -n 5 "$REMOTE/pure_linux.log"
tail -n 5 "$REMOTE/real_ros_callbacks.log"
