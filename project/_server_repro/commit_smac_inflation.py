from pathlib import Path
import os
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
paths = [
    'nav2_adapter/config/nav2_ackermann_control.yaml',
    'nav2_adapter/wildos_nav2_adapter/runtime_contract.py',
    'nav2_adapter/test/test_runtime_contract.py',
    'tools/cpp_navigation_probe/CMakeLists.txt',
    'tools/cpp_navigation_probe/collision_snapshot_probe.cpp',
    'tools/cpp_navigation_probe/smac_planner_snapshot_probe.cpp',
    'tools/cpp_navigation_probe/README.md',
    'tools/test_smac_snapshot_evidence.py',
    'docs/PROJECT_MEMORY.md',
    'test_artifacts/server_validation_20261010/PILOT3_SMAC_INFLATION_REPAIR.md',
    'test_artifacts/server_validation_20261010/smac_boundary_evidence.zip',
    'test_artifacts/offline_navigation/smac_inflation_contract.log',
]

def git(*args, env=None):
    return subprocess.check_output(['git', *args], cwd=root, env=env)

def unrelated():
    return [r for r in git('ls-files', '--stage', '-z').split(b'\0') if r and
            r.split(b'\t', 1)[1].decode('utf-8') not in paths]

before = unrelated()
with tempfile.TemporaryDirectory(dir=root/'_server_repro') as temporary:
    env = os.environ.copy()
    env['GIT_INDEX_FILE'] = str(Path(temporary)/'index')
    git('read-tree', 'HEAD', env=env)
    git('add', '--', *paths, env=env)
    git('-c', 'core.whitespace=blank-at-eol,blank-at-eof,space-before-tab,cr-at-eol',
        'diff', '--cached', '--check', '--',
        *[p for p in paths if not p.endswith(('.zip', '.log'))], env=env)
    print(git('commit', '-m', 'fix: cover Smac rectangular collision shortcut with global inflation',
              env=env).decode())
for path in paths:
    entry = git('ls-tree', 'HEAD', '--', path).decode().split('\t', 1)[0].split()
    git('update-index', '--add', '--cacheinfo', ','.join((entry[0], entry[2], path)))
assert before == unrelated(), 'Unrelated staged entries changed'
print('COMMIT', git('rev-parse', 'HEAD').decode().strip())
