"""Build a separate field harness from the last verified single-target harness."""
import ast,hashlib,json,zipfile
from pathlib import Path
out=Path(__file__).resolve().parent;root=out.parents[1]
old=root/'test_artifacts/frontier_motion_handoff_20261016'
previous='/home/ly/frontier_motion_handoff_20261016'
remote='/home/ly/source_control_repair_20261017'
digest=json.loads((root/'official_wildos_beamng_bundle.manifest.json').read_text(encoding='utf8'))['sha256']
old_digest='12023042037205a79a57f0e76bb1d8544e94309f8ff9b80a3fe28f554f521f2d'
field=out/'field';field.mkdir(exist_ok=True)
for src in (old/'field').iterdir():
    if src.suffix not in ('.py','.sh','.json','.yaml'):continue
    value=src.read_text(encoding='utf8').replace(previous,remote).replace(old_digest,digest)
    if src.name=='sensor_full_console.json':
        data=json.loads(value);data['setups'].insert(-1,previous+'/install/setup.bash')
        value=json.dumps(data,indent=2)+'\n'
    if src.name=='sensor_full_console.sh':
        value=value.replace('source '+remote+'/install/setup.bash',
            'source '+previous+'/install/setup.bash\nsource '+remote+'/install/setup.bash')
    if src.name=='record_execution_contract.py':
        value=value.replace("(String,'/wildos/control_status'),", "(String,'/wildos/native_control_command'),(String,'/wildos/execution_hold'),\n    (String,'/wildos/control_status'),")
    if src.name=='drive_sensor_full.py':
        value=value.replace("    send('/status')\n    if args.stage", "    send('/status')\n    component=subprocess.run(['bash',str(root/'field/probe_source.sh'),str(out/'source_lifecycle.json')],stdout=(out/'source_probe.log').open('x'),stderr=subprocess.STDOUT,timeout=40)\n    assert component.returncode==0, 'source lifecycle component failed; see source_probe.log'\n    report['source_lifecycle']=json.loads((out/'source_lifecycle.json').read_text())\n    if args.stage")
    if src.suffix=='.py':ast.parse(value)
    (field/src.name).write_text(value,encoding='utf8',newline='\n')
for name in ('live_summary.py','review_field.py','inspect_closed_progress.py','archive_field.py','close_and_archive.sh'):
    src=old/name
    if src.exists():(out/name).write_text(src.read_text(encoding='utf8').replace(previous,remote).replace(old_digest,digest),encoding='utf8',newline='\n')
config=json.loads((field/'sensor_full_console.json').read_text())
setups='\n'.join('source '+p for p in config['setups'])
probe='#!/usr/bin/env bash\nset -eo pipefail\n'+setups+'\nexport ROS_DOMAIN_ID=88\n'
python='/home/ly/beamng_ros2_0394_ws/.venv/bin/python'
probe+='exec '+python+' -m wildos_nav2_adapter.dds_profile --profile '+remote+'/candidate/nav2_adapter/config/fastdds_local_udp.xml -- '+python+' '+remote+'/field/probe_source_lifecycle.py "$1"\n'
(field/'probe_source.sh').write_text(probe,encoding='utf8',newline='\n')
build='''#!/usr/bin/env bash
set -eo pipefail
export PATH="$HOME/.local/bin:$PATH"
REMOTE="REMOTE_VALUE"
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
SETUPS_VALUE
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
'''.replace('REMOTE_VALUE',remote).replace('SETUPS_VALUE','\n'.join('source '+p for p in config['setups'][:-1]))
(out/'build_and_test.sh').write_text(build,encoding='utf8',newline='\n')
runner='#!/usr/bin/env bash\nset -eo pipefail\n'+setups+'\n'
runner+='export ROS_DOMAIN_ID=88 DISPLAY=:148\n'
runner+='python3 '+remote+'/field/verify_before.py\n'
runner+='python3 '+remote+'/field/drive_sensor_full.py search\n'
(out/'run_field.sh').write_text(runner,encoding='utf8',newline='\n')
harness=json.loads((field/'harness_manifest.json').read_text())
harness['candidate_sha256']=digest
harness['files']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in field.iterdir()
    if p.name!='harness_manifest.json' and p.suffix in ('.py','.sh','.json','.yaml')}
(field/'harness_manifest.json').write_text(json.dumps(harness,indent=2)+'\n',encoding='utf8')
with zipfile.ZipFile(out/'harness.zip','w',zipfile.ZIP_DEFLATED) as archive:
    files=[p for p in field.iterdir() if p.suffix in ('.py','.sh','.json','.yaml')]
    files += [out/n for n in ('live_summary.py','review_field.py','inspect_closed_progress.py','archive_field.py','close_and_archive.sh')]
    for p in files:
        if p.suffix=='.py':ast.parse(p.read_text(encoding='utf8'))
        archive.write(p,p.relative_to(out).as_posix())
print(remote,digest)
