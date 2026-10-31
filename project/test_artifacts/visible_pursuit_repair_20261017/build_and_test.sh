#!/usr/bin/env bash
set -eo pipefail
REMOTE=/home/ly/visible_pursuit_repair_20261017
python3 - <<'PY'
from pathlib import Path
import ast,hashlib,json,zipfile
r=Path('/home/ly/visible_pursuit_repair_20261017')
m=json.loads((r/'official_wildos_beamng_bundle.manifest.json').read_text())
assert m['sha256']=='f9d1caa748373c8bd9106cde7f44105604d069a3b667b9dd3279b47b4d5defec'
assert hashlib.sha256((r/'official_wildos_beamng_bundle.zip').read_bytes()).hexdigest()==m['sha256']
assert hashlib.sha256((r/'harness.zip').read_bytes()).hexdigest()=='ef7e5e4941a19f7dce1896a33d21261062413f66706c0fabe2653fadb1b096d4'
for filename,target in [('official_wildos_beamng_bundle.zip',r/'candidate'),('harness.zip',r)]:
    with zipfile.ZipFile(r/filename) as z:
        assert z.testzip() is None
        assert all(not Path(n).is_absolute() and '..' not in Path(n).parts for n in z.namelist())
        if filename!='harness.zip':
            assert set(z.namelist())==set(m['members'])
            for n,h in m['members'].items():assert hashlib.sha256(z.read(n)).hexdigest()==h,n
        z.extractall(target)
for p in (r/'field').glob('*.py'):ast.parse(p.read_text())
(r/'manifest.json').write_text(json.dumps(m,indent=2))
print('source_and_harness_verified',m['sha256'],len(m['members']))
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
/home/ly/beamng_ros2_0394_ws/.venv/bin/colcon --log-base "$REMOTE/build_logs" build --base-paths "$REMOTE/candidate/nav2_adapter" "$REMOTE/candidate/official_adapter" --packages-select wildos_nav2_adapter beamng_wildos_adapter --build-base "$REMOTE/build" --install-base "$REMOTE/install" --merge-install > "$REMOTE/build.log" 2>&1
source "$REMOTE/install/setup.bash"
export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1
export PYTHONPATH="$REMOTE/candidate/nav2_adapter/test:$REMOTE/candidate/tools:$REMOTE/candidate/official_adapter:$REMOTE/candidate/wildos_port/repro_check/src/nebula2-wildos/visual_navigation:$PYTHONPATH"
python3 -m unittest test_source_control test_source_control_preflight test_dds_profile test_runtime_contract -v > "$REMOTE/pure_linux.log" 2>&1
python3 - > "$REMOTE/real_ros_callbacks.log" 2>&1 <<'PY'
import rclpy,unittest
rclpy.init()
try:
    suite=unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromName(n) for n in
        ('test_source_control_ros','test_native_vehicle_control_ros',
         'test_planner_execution_ownership_ros.PlannerOwnershipTests','test_visible_approach_ros'))
    result=unittest.TextTestRunner(verbosity=2).run(suite)
finally:rclpy.shutdown()
raise SystemExit(not result.wasSuccessful())
PY
python3 -m unittest test_graph_path_bridge_routing.ExplorationTargetHandoffTests test_exploration_cost_ros -v > "$REMOTE/handoff_cost_callbacks.log" 2>&1
python3 "$REMOTE/candidate/tools/check_physical_launch.py" --output "$REMOTE/launch_expansion.json" > "$REMOTE/launch_expansion.log" 2>&1
/home/ly/beamng_ros2_0394_ws/.venv/bin/python - <<'PY'
import importlib,hashlib,json
from pathlib import Path
r=Path('/home/ly/visible_pursuit_repair_20261017');m=json.loads((r/'manifest.json').read_text());report={}
for package,names,prefix in (
 ('wildos_nav2_adapter',('source_control','cmd_vel_to_control','native_vehicle_control','graph_path_bridge','visible_approach','runtime_contract'),'nav2_adapter'),
 ('beamng_wildos_adapter',('goal_standoff',),'official_adapter')):
    for name in names:
        p=Path(importlib.import_module(package+'.'+name).__file__)
        h=hashlib.sha256(p.read_bytes()).hexdigest()
        assert h==m['members'][prefix+'/'+package+'/'+p.name],p
        report[package+'.'+name]=dict(path=str(p),sha256=h)
(r/'installed_identity.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
PY
python3 "$REMOTE/field/verify_before.py" > "$REMOTE/prelaunch_source_check.json"
tail -n 5 "$REMOTE/pure_linux.log"
tail -n 5 "$REMOTE/real_ros_callbacks.log"
tail -n 5 "$REMOTE/handoff_cost_callbacks.log"
