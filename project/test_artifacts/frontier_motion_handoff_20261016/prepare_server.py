from pathlib import Path
import ast, json, hashlib
root = Path(__file__).resolve().parents[2]
out = Path(__file__).parent
old = root/'test_artifacts/frontier_resolution_repair_20261016'
previous = '/home/ly/frontier_resolution_repair_20261016'
remote = '/home/ly/frontier_motion_handoff_20261016'
digest = json.loads((out/'delta.json').read_text())['candidate']
old_digest = json.loads((out/'delta.json').read_text())['base']
(out/'field').mkdir(exist_ok=True)
for src in (old/'field').iterdir():
    if src.suffix not in ('.py', '.sh', '.json', '.yaml'): continue
    value = src.read_text(encoding='utf8').replace(previous, remote).replace(old_digest, digest)
    if src.name == 'sensor_full_console.json':
        data = json.loads(value)
        data['setups'].insert(-1, previous+'/install/setup.bash')
        value = json.dumps(data, indent=2)+'\n'
    if src.name == 'sensor_full_console.sh':
        value = value.replace('source '+remote+'/install/setup.bash',
            'source '+previous+'/install/setup.bash\nsource '+remote+'/install/setup.bash')
    if src.suffix == '.py': ast.parse(value)
    (out/'field'/src.name).write_text(value, encoding='utf8', newline='\n')
for name in ('live_summary.py','review_field.py','inspect_closed_progress.py','archive_field.py'):
    src = old/name
    if src.exists():
        value = src.read_text(encoding='utf8').replace(previous,remote).replace(old_digest,digest)
        ast.parse(value)
        (out/name).write_text(value,encoding='utf8',newline='\n')
config=json.loads((out/'field/sensor_full_console.json').read_text())
report=json.loads((out/'field/harness_manifest.json').read_text())
report['files']={p.name:hashlib.sha256(p.read_bytes()).hexdigest()
    for p in (out/'field').iterdir() if p.name!='harness_manifest.json'
    and p.suffix in ('.py','.sh','.yaml','.json')}
(out/'field/harness_manifest.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf8')
setups='\n'.join('source '+p for p in config['setups'])
script='''#!/usr/bin/env bash
set -eo pipefail
export PATH="$HOME/.local/bin:$PATH"
REMOTE="REMOTE_VALUE"
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
SETUPS_VALUE
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
'''.replace('REMOTE_VALUE',remote).replace('SETUPS_VALUE',setups.replace('source '+remote+'/install/setup.bash',''))
(out/'build_and_test.sh').write_text(script,encoding='utf8',newline='\n')
print(remote,digest)
