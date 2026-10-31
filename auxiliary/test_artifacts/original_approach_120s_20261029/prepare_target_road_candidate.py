"""Prepare an isolated one-runtime-file candidate; preserve the v66 baseline."""
import hashlib
import json
from pathlib import Path
import shutil

parent = Path('/home/ly/execution_continuity_review_20261022')
old = parent / 'v66_original_approach_120s'
new = parent / 'v67_target_road_120s'
assert not new.exists(), new
new.mkdir()
shutil.copytree(old / 'candidate', new / 'candidate', symlinks=True,
                ignore=shutil.ignore_patterns('__pycache__'))
(new / 'full').mkdir()
for path in (old / 'full').iterdir():
    if path.is_file() and path.suffix in ('.py', '.json'):
        target = new / 'full' / path.name
        target.write_text(path.read_text().replace(str(old), str(new)))
(new / 'full/candidate').symlink_to('../candidate', target_is_directory=True)
changed = 'nav2_adapter/wildos_nav2_adapter/pointcloud.py'
shutil.copy2(old / 'pointcloud_fast_review.py', new / 'candidate' / changed)
tests = parent / 'v64_ground_surface/candidate/nav2_adapter/test'
shutil.copy2(tests / 'test_curved_scan_ground.py', new / 'candidate/nav2_adapter/test/test_curved_scan_ground.py')
shutil.copytree(tests / 'fixtures/curved_scan_ground', new / 'candidate/nav2_adapter/test/fixtures/curved_scan_ground')
config = json.loads((new / 'full/config.json').read_text())
# The new overlay builds only Python packages: retain the v66 GraphNav library.
config['setups'] = [str(old / 'install/local_setup.bash') if x == str(new / 'install/local_setup.bash') else x
                    for x in config['setups']]
config['setups'].append(str(new / 'install/local_setup.bash'))
assert config['goal_timeout_s'] == 120
(new / 'full/config.json').write_text(json.dumps(config, indent=2) + '\n')
lines = (old / 'run_full.sh').read_text().splitlines()
last_source = max(i for i, line in enumerate(lines) if line.startswith('source '))
lines.insert(last_source + 1, f'source {new}/install/local_setup.bash')
for i, line in enumerate(lines):
    if not line.startswith('source '):
        lines[i] = line.replace(str(old), str(new))
(new / 'run_full.sh').write_text('\n'.join(lines) + '\n')
manifest = json.loads((old / 'full/frozen_source_manifest.json').read_text())
differences = []
for name, digest in manifest['members'].items():
    actual = hashlib.sha256((new / 'candidate' / name).read_bytes()).hexdigest()
    if digest != actual:
        differences.append(name)
    manifest['members'][name] = actual
assert differences == [changed], differences
added = ['nav2_adapter/test/test_curved_scan_ground.py']
added += [str(p.relative_to(new / 'candidate')) for p in
          (new / 'candidate/nav2_adapter/test/fixtures/curved_scan_ground').iterdir() if p.is_file()]
for name in added:
    manifest['members'][name] = hashlib.sha256((new / 'candidate' / name).read_bytes()).hexdigest()
(new / 'full/frozen_source_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
runner = (old / 'isolated_ros_review.py').read_text().replace(
    "root=Path.cwd()/'isolated_ros'", "modules=tuple(modules)+('test_curved_scan_ground',)\nroot=Path.cwd()/'isolated_ros'")
(new / 'isolated_ros_review.py').write_text(runner)
for script in ('audit_fluency_120s.py', 'audit_cloud_surface_v66.py', 'compare_static_surface_v66.py'):
    if (old / script).exists():
        (new / script).write_text((old / script).read_text().replace(str(old), str(new)))
report = dict(runtime_delta=differences, tests_added=added, members=len(manifest['members']),
              search_s=120, baseline=str(old), no_new_runtime_components=True,
              graph_library=config['library_sha256']['libgraphnav_planner_component.so'])
(new / 'candidate_scope.json').write_text(json.dumps(report, indent=2))
print(json.dumps(report, indent=2))
