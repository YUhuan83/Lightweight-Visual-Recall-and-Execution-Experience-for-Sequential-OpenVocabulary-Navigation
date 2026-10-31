"""Clone the frozen v10 trial without copying any old trial outputs."""
from pathlib import Path
import hashlib
import json
import shutil
import zipfile

BASE = Path('/home/ly/execution_continuity_repair_20261021/v10')
DEST = BASE.with_name('reachable_observation_v11')
INCOMING = BASE.parent / 'reachable_observation_incoming'
plan = json.loads((INCOMING/'deployment_plan.json').read_text())
assert not DEST.exists(), str(DEST)
frozen = json.loads((BASE/'full/frozen_source_manifest.json').read_text())
mismatches = [n for n,h in frozen['members'].items()
              if hashlib.sha256((BASE/'candidate'/n).read_bytes()).hexdigest() != h]
assert not mismatches, mismatches
for name, expected in plan['baseline_normalized_sha256'].items():
    actual = hashlib.sha256((BASE/'candidate'/name).read_text().encode()).hexdigest()
    assert actual == expected, ('Changed server baseline', name, actual, expected)
archive = INCOMING/'candidate_delta.zip'
assert hashlib.sha256(archive.read_bytes()).hexdigest() == plan['archive_sha256']
with zipfile.ZipFile(archive) as bundle:
    assert bundle.testzip() is None
    delta = json.loads(bundle.read('delta_manifest.json'))
    for name, meta in delta.items():
        assert not Path(name).is_absolute() and '..' not in Path(name).parts
        assert hashlib.sha256(bundle.read(name)).hexdigest() == meta['sha256']
    DEST.mkdir()
    shutil.copytree(BASE/'candidate', DEST/'candidate', symlinks=True)
    (DEST/'full').mkdir()
    for file in (BASE/'full').iterdir():
        if file.is_file() and file.suffix == '.py':
            shutil.copy2(file, DEST/'full'/file.name)
    (DEST/'full/candidate').symlink_to(DEST/'candidate', target_is_directory=True)
    for name in delta:
        target = DEST/'candidate'/name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(bundle.read(name))
        frozen['members'][name] = delta[name]['sha256']
config = json.loads((BASE/'full/config.json').read_text())
config['setups'].append(str(DEST/'install/local_setup.bash'))
config['python_paths'].insert(0, str(DEST/'candidate/wildos_port/repro_check/src/nebula2-wildos'))
(DEST/'full/config.json').write_text(json.dumps(config, indent=2)+'\n')
(DEST/'full/frozen_source_manifest.json').write_text(json.dumps(frozen, indent=2)+'\n')
(DEST/'deployment_receipt.json').write_text(json.dumps(dict(
    base=str(BASE), archive_sha256=plan['archive_sha256'], changed=delta,
    source_members=len(frozen['members']), baseline_mismatches=mismatches), indent=2)+'\n')
shell = (BASE/'run_full.sh').read_text()
shell = shell.replace(str(BASE), str(DEST))
(DEST/'run_full.sh').write_text(shell)
build = '''#!/usr/bin/env bash
set -eo pipefail
source <(sed '/^exec /,$d' /home/ly/execution_continuity_repair_20261021/v10/run_full.sh)
export PATH=/home/ly/beamng_ros2_0394_ws/.venv/bin:$PATH
cd REPLACE_DEST
colcon --log-base build_log build --base-paths candidate/nav2_adapter candidate/official_adapter --packages-select wildos_nav2_adapter beamng_wildos_adapter --build-base build --install-base install --parallel-workers 2 >build.log 2>&1
source install/local_setup.bash
export ROS_DOMAIN_ID=204 ROS_LOCALHOST_ONLY=1
export PYTHONPATH=$PWD/candidate/nav2_adapter/test:$PYTHONPATH
/home/ly/beamng_ros2_0394_ws/.venv/bin/python -m unittest -v test_reachable_observation_ros >reachable_ros.log 2>&1
/home/ly/beamng_ros2_0394_ws/.venv/bin/python -m unittest -v test_visible_approach_ros >visible_ros.log 2>&1
/home/ly/beamng_ros2_0394_ws/.venv/bin/python - <<'PY' >installed_identity.json
import hashlib,json
from pathlib import Path
from beamng_wildos_adapter import goal_standoff,mapping,visible_approach
from wildos_nav2_adapter import graph_path_bridge,observation_pose,target_observation
mods=[goal_standoff,mapping,visible_approach,graph_path_bridge,observation_pose,target_observation]
manifest=json.loads(Path('full/frozen_source_manifest.json').read_text())['members']
out={}
for mod in mods:
    p=Path(mod.__file__).resolve();h=hashlib.sha256(p.read_bytes()).hexdigest()
    assert str(Path.cwd()/'install') in str(p), str(p)
    assert h in manifest.values(), str(p)
    out[mod.__name__]=dict(path=str(p),sha256=h)
print(json.dumps(out,indent=2))
PY
'''.replace('REPLACE_DEST',str(DEST))
(DEST/'build.sh').write_text(build)
print(json.dumps(dict(destination=str(DEST), files=len(delta), members=len(frozen['members']))))
