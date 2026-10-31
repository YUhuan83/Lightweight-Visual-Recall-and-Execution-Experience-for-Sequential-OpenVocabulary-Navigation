"""Deploy only the reviewed obstacle contract after disabling motion."""
from pathlib import Path
import hashlib
import json
import os
import signal
import subprocess
import sys
import time
import zipfile

ARM = Path('/home/ly/wildos_ab_20261008_B_fix')
SESSION = Path('/home/ly/town_memory_validation_20261008')
OUT = Path('/home/ly/town_memory_validation_20261009/obstacle_restore')
FILES = ('nav2_adapter/config/nav2_official_control.yaml',
         'tools/offline_navigation_check.py', 'tools/server_test_preflight.py')


def run(args, **kwargs):
    return subprocess.run(args, cwd=ARM, check=True, **kwargs)


def main():
    OUT.mkdir(exist_ok=True)
    previous = json.loads((OUT.parent/'preflight_before_obstacle_restore.json').read_text())
    assert previous['controls_before_test'] and not any(previous['controls_before_test'].values())
    assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ARM, text=True).strip() == 'd62752a582fd92a9f11db8439d12ab29aa9db2b4'
    changes = subprocess.check_output(['git', 'status', '--porcelain', '--untracked-files=no'], cwd=ARM, text=True).splitlines()
    assert all(row[3:] in FILES for row in changes), 'Unrelated production changes'
    with zipfile.ZipFile(OUT/'candidate.zip') as candidate:
        for row in changes:
            name = row[3:]
            assert (ARM/name).read_bytes() == candidate.read(name).replace(b'\r\n', b'\n'), 'Unexpected change: '+name
    stopped = []
    for role in ('lifecycle', 'controller', 'planner'):
        rows = previous['roles'][role]
        assert len(rows) == 1
        row = rows[0]
        pid = row['pid']
        proc = Path('/proc')/str(pid)/'cmdline'
        if proc.exists() and proc.read_bytes():
            tokens = [t.decode() for t in proc.read_bytes().split(b'\0') if t]
            assert tokens == row['tokens'], 'Process identity changed: '+role
            os.kill(pid, signal.SIGINT)
            until = time.monotonic()+10
            while proc.exists() and proc.read_bytes() and time.monotonic() < until:
                time.sleep(.1)
            if proc.exists() and proc.read_bytes():
                os.kill(pid, signal.SIGTERM)
                until = time.monotonic()+5
                while proc.exists() and proc.read_bytes() and time.monotonic() < until:
                    time.sleep(.1)
            assert not proc.exists() or not proc.read_bytes(), 'Previous process did not exit'
        stopped.append({'role': role, 'pid': pid, 'command': row['command']})
    manifest = {'base_commit': 'd62752a582fd92a9f11db8439d12ab29aa9db2b4',
                'stopped': stopped, 'files': {}, 'production_changes': list(FILES)}
    with zipfile.ZipFile(OUT/'candidate.zip') as bundle:
        assert set(bundle.namelist()) == set(FILES)
        for name in FILES:
            path = ARM/name
            backup = OUT/'before'/name
            backup.parent.mkdir(parents=True, exist_ok=True)
            if path.exists() and not backup.exists():
                backup.write_bytes(path.read_bytes())
            data = bundle.read(name).replace(b'\r\n', b'\n')
            path.write_bytes(data)
            manifest['files'][name] = hashlib.sha256(data).hexdigest()
    run(['git', 'add', '--', *FILES])
    run(['git', '-c', 'user.name=Codex', '-c', 'user.email=codex@local.invalid',
         'commit', '-m', 'Restore obstacle constraints independently of ideal carrier dynamics'])
    manifest['commit'] = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ARM, text=True).strip()
    (OUT/'deployment.json').write_text(json.dumps(manifest, indent=2))
    print('SOURCE_FROZEN', manifest['commit'], flush=True)
    with (OUT/'build.log').open('w') as log:
        run(['/home/ly/beamng_ros2_0394_ws/.venv/bin/colcon', 'build',
             '--base-paths', 'nav2_adapter', '--packages-select', 'wildos_nav2_adapter',
             '--symlink-install'], stdout=log, stderr=subprocess.STDOUT, timeout=180)
    print('BUILD_PASSED', flush=True)
    # Runner preflight must use the same current gate as the committed source.
    (SESSION/'server_test_preflight.py').write_bytes((ARM/'tools/server_test_preflight.py').read_bytes())
    for role in ('planner', 'controller', 'lifecycle'):
        result = run([sys.executable, str(SESSION/'restart_component.py'), role],
                     text=True, capture_output=True, timeout=20)
        print(result.stdout.strip(), flush=True)
    manifest['build_passed'] = True
    manifest['runtime_restarted'] = True
    (OUT/'deployment.json').write_text(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
