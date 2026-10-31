"""Create an isolated candidate from verified v11 sources; retain old trials."""
from pathlib import Path
import hashlib
import json
import shutil
import zipfile

BASE = Path('/home/ly/execution_continuity_repair_20261021/reachable_observation_v11')
ROOT = Path('/home/ly/execution_continuity_review_20261022')
DEST = ROOT / 'v1'
assert not DEST.exists(), str(DEST)
frozen = json.loads((BASE / 'full/frozen_source_manifest.json').read_text())
assert all(hashlib.sha256((BASE / 'candidate' / n).read_bytes()).hexdigest() == h
           for n, h in frozen['members'].items())
plan = json.loads((ROOT / 'incoming/deployment_plan.json').read_text())
archive = ROOT / 'incoming/candidate_delta.zip'
assert hashlib.sha256(archive.read_bytes()).hexdigest() == plan['archive_sha256']
DEST.mkdir()
shutil.copytree(BASE / 'candidate', DEST / 'candidate', symlinks=True)
(DEST / 'full').mkdir()
for source in (BASE / 'full').glob('*.py'):
    shutil.copy2(source, DEST / 'full' / source.name)
(DEST / 'full/candidate').symlink_to(DEST / 'candidate', target_is_directory=True)
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None
    for name, expected in plan['members'].items():
        data = z.read(name)
        assert hashlib.sha256(data).hexdigest() == expected
        target = DEST / 'candidate' / name
        assert target.resolve().is_relative_to((DEST / 'candidate').resolve())
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        frozen['members'][name] = expected
(DEST / 'full/frozen_source_manifest.json').write_text(json.dumps(frozen, indent=2))
config = json.loads((BASE / 'full/config.json').read_text())
config['setups'].append(str(DEST / 'install/local_setup.bash'))
config['python_paths'].insert(0, str(DEST / 'candidate/wildos_port/repro_check/src/nebula2-wildos'))
config['goal_timeout_s'] = 420
config['output_root'] = str(DEST / 'full/sessions')
(DEST / 'full/config.json').write_text(json.dumps(config, indent=2))
shell = (BASE / 'run_full.sh').read_text().replace(str(BASE), str(DEST))
(DEST / 'run_full.sh').write_text(shell)
shutil.copy2(ROOT / 'incoming/build_candidate.sh', DEST / 'build.sh')
shutil.copy2(ROOT / 'incoming/real_ros_review.py', DEST / 'real_ros_review.py')
(DEST / 'deployment_receipt.json').write_text(json.dumps(plan, indent=2))
print(json.dumps({'destination': str(DEST), 'updated': len(plan['members']),
                  'source_members': len(frozen['members'])}))
