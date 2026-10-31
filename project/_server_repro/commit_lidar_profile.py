"""Commit only this trial's records; preserve unrelated staged entries."""
from pathlib import Path
import os
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
paths = ['config/sbr_physical_sensor_rig.json', 'tools/test_physical_sensor_rig.py', 'tools/test_lidar_chain_evidence.py', 'docs/PHYSICAL_LIDAR_REPAIR_20261011.md', 'docs/PROJECT_MEMORY.md', 'test_artifacts/server_validation_20261011/lidar_chain_evidence.zip', 'test_artifacts/server_validation_20261011/lidar_chain_tests.log']


def git(*args, env=None):
    return subprocess.check_output(['git', *args], cwd=root, env=env)


def unrelated():
    return [r for r in git('ls-files', '--stage', '-z').split(b'\0') if r and
            r.split(b'\t', 1)[1].decode('utf-8') not in paths]


before = unrelated()
with tempfile.TemporaryDirectory(dir=root / '_server_repro') as temporary:
    env = os.environ.copy()
    env['GIT_INDEX_FILE'] = str(Path(temporary) / 'index')
    git('read-tree', 'HEAD', env=env)
    git('add', '--', *paths, env=env)
    git('-c', 'core.whitespace=blank-at-eol,blank-at-eof,space-before-tab,cr-at-eol',
        'diff', '--cached', '--check', env=env)
    print(git('commit', '-m', 'fix: correct physical 360 lidar coverage and preserve scan controls', env=env).decode())
for path in paths:
    entry = git('ls-tree', 'HEAD', '--', path).decode().split('\t', 1)[0].split()
    git('update-index', '--add', '--cacheinfo', ','.join((entry[0], entry[2], path)))
assert before == unrelated(), 'Unrelated staged entries changed'
print('COMMIT', git('rev-parse', 'HEAD').decode().strip())
