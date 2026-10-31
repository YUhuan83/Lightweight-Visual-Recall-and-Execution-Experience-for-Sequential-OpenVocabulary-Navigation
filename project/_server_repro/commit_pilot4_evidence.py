"""Commit only this trial's records; preserve unrelated staged entries."""
from pathlib import Path
import os
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
paths = [
    'docs/PROJECT_MEMORY.md', 'tools/test_pilot4_failure_evidence.py',
    'test_artifacts/server_validation_20261011/PILOT4.md',
    'test_artifacts/server_validation_20261011/pilot4_closed_evidence.zip',
    'test_artifacts/server_validation_20261011/pilot4_evidence_tests.log',
    'test_artifacts/server_validation_20261011/pilot4_evidence_tests_attempt1.log',
]


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
    print(git('commit', '-m', 'test: preserve physical pilot4 obstacle visibility failure', env=env).decode())
for path in paths:
    entry = git('ls-tree', 'HEAD', '--', path).decode().split('\t', 1)[0].split()
    git('update-index', '--add', '--cacheinfo', ','.join((entry[0], entry[2], path)))
assert before == unrelated(), 'Unrelated staged entries changed'
print('COMMIT', git('rev-parse', 'HEAD').decode().strip())
