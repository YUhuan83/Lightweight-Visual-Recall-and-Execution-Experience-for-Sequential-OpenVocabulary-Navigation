"""Apply the reviewed candidate files to the frozen server baseline, then build.

Run on the authorized ROS server. This never starts motion or kills processes.
"""
from pathlib import Path
import hashlib
import json
import subprocess
import time

LIVE = Path('/home/ly/wildos_ab_20261008_B_fix')
AUDIT = Path('/home/ly/town_memory_validation_20261008/stability_audit_20261009')
CANDIDATE = AUDIT / 'candidate_source'
OUT = Path('/home/ly/town_memory_validation_20261009')
BASE = '70c4b2e6e25015f01b9758a6f75b7d023d95004f'
FILES = [
    'nav2_adapter/wildos_nav2_adapter/sensor_geometry.py',
    'nav2_adapter/wildos_nav2_adapter/target_observation.py',
    'nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py',
    'nav2_adapter/wildos_nav2_adapter/visible_approach.py',
    'nav2_adapter/wildos_nav2_adapter/memory_selection.py',
    'nav2_adapter/wildos_nav2_adapter/confirmed_approach.py',
    'nav2_adapter/test/test_sensor_observation_lifecycle_ros.py',
    'nav2_adapter/test/test_goal_confirmation_ros.py',
    'nav2_adapter/test/test_multi_goal_session_ros.py',
    'official_adapter/beamng_wildos_adapter/node.py',
    'official_adapter/beamng_wildos_adapter/goal_standoff.py',
    'official_adapter/beamng_wildos_adapter/geometry.py',
    'official_adapter/beamng_wildos_adapter/mapping.py',
    'official_adapter/package.xml',
    'tools/offline_callback_check.py',
    'tools/server_test_preflight.py',
    'tools/audit_stability_boundaries.py',
    'wildos_port/repro_check/src/nebula2-wildos/graphnav_planner/src/planner.cpp',
    'wildos_port/repro_check/src/nebula2-wildos/graphnav_planner/test/test_route_mechanism.cpp',
]


def git(*args):
    return subprocess.check_output(['git', '-C', str(LIVE), *args], text=True).strip()


def main():
    assert git('rev-parse', 'HEAD') == BASE, 'Unexpected baseline'
    branch = git('branch', '--show-current')
    dirty = git('diff', '--name-only', 'HEAD').splitlines()
    untracked = git('ls-files', '--others', '--exclude-standard').splitlines()
    if dirty or untracked:
        assert branch == 'experiment/geometry-lifecycle-20261009'
        assert set(dirty + untracked).issubset(FILES), 'Unexpected rollout changes'
        assert all((LIVE/name).read_bytes() == (CANDIDATE/name).read_bytes()
                   for name in dirty + untracked), 'Candidate drift during resume'
    OUT.mkdir(exist_ok=True)
    report = {'base_commit': BASE, 'files': {}, 'build_passed': False,
              'runtime_restarted': False, 'motion_enabled': False}
    for name in FILES:
        src, dst = CANDIDATE/name, LIVE/name
        assert src.is_file() and src.resolve().is_relative_to(CANDIDATE.resolve())
        assert dst.resolve().is_relative_to(LIVE.resolve())
        if src.suffix == '.py':
            compile(src.read_text(), str(src), 'exec')
        old = subprocess.run(['git', '-C', str(LIVE), 'show', BASE+':'+name], capture_output=True)
        report['files'][name] = {'candidate_sha256': hashlib.sha256(src.read_bytes()).hexdigest(),
            'baseline_sha256': hashlib.sha256(old.stdout).hexdigest() if old.returncode == 0 else None}
    vision = LIVE/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/wildos/nav.py'
    report['unchanged_vision_sha256'] = hashlib.sha256(vision.read_bytes()).hexdigest()
    (OUT/'deployment_manifest.json').write_text(json.dumps(report, indent=2))
    if branch != 'experiment/geometry-lifecycle-20261009':
        git('checkout', '-b', 'experiment/geometry-lifecycle-20261009')
    for name in FILES:
        dst = LIVE/name
        dst.parent.mkdir(parents=True, exist_ok=True)
        dst.write_bytes((CANDIDATE/name).read_bytes())
    assert hashlib.sha256(vision.read_bytes()).hexdigest() == report['unchanged_vision_sha256']
    git('add', '--', *FILES)
    git('-c', 'user.name=Codex', '-c', 'user.email=codex@localhost',
        'commit', '-m', 'Fix sensor observation geometry and target lifecycle handover')
    report['candidate_commit'] = git('rev-parse', 'HEAD')
    report['diff_stat'] = git('diff', '--stat', BASE, 'HEAD')
    (OUT/'deployment_manifest.json').write_text(json.dumps(report, indent=2))
    cmd = '\n'.join([
        'set -e',
        'source /opt/ros/humble/setup.bash',
        'source /home/ly/beamng_ros2_0394_ws/install/setup.bash',
        'source /home/ly/wildos_ab_20261008_B_fix/install/setup.bash',
        'export CMAKE_BUILD_PARALLEL_LEVEL=2',
        'exec /home/ly/beamng_ros2_0394_ws/.venv/bin/colcon build '
        '--base-paths nav2_adapter official_adapter '
        'wildos_port/repro_check/src/nebula2-wildos/graphnav_planner '
        '--packages-select wildos_nav2_adapter beamng_wildos_adapter graphnav_planner '
        '--symlink-install --cmake-args -DBUILD_TESTING=ON',
    ])
    started = time.monotonic()
    with (OUT/'build.log').open('w') as log:
        result = subprocess.run(['/bin/bash', '-c', cmd], cwd=LIVE,
                                stdout=log, stderr=subprocess.STDOUT)
    report.update(build_returncode=result.returncode, build_passed=result.returncode == 0,
                  build_elapsed_s=time.monotonic()-started)
    (OUT/'deployment_manifest.json').write_text(json.dumps(report, indent=2))
    print(json.dumps({k:report[k] for k in ('candidate_commit','build_passed','build_returncode')}), flush=True)
    raise SystemExit(result.returncode)


if __name__ == '__main__':
    main()
