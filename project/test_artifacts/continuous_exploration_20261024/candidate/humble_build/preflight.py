"""Read-only inventory/environment checks; explicit isolated-copy preparation."""
import argparse
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys
import sysconfig
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
DEFAULT_SOURCE = HERE.parent / 'wildos_port/workspace/src/nebula2-wildos'
SKIP = {'.git', '.venv', 'venv', '__pycache__', '.pytest_cache', '.mypy_cache',
        '.ruff_cache', 'build', 'install', 'log', 'logs', 'node_modules',
        '.cache', 'weights', 'checkpoints'}
SUFFIXES = {'.pyc', '.pyo', '.pt', '.pth', '.onnx', '.safetensors', '.ckpt'}


def files(source):
    for base, dirs, names in os.walk(source, followlinks=False):
        dirs[:] = sorted(d for d in dirs if d not in SKIP)
        for name in dirs + sorted(names):
            path = Path(base) / name
            if path.is_symlink():
                raise ValueError('Refusing source symlink: ' + str(path))
        for name in sorted(names):
            path = Path(base) / name
            if path.suffix not in SUFFIXES:
                if not path.is_file():
                    raise ValueError('Not a regular file: ' + str(path))
                yield path


def snapshot(source):
    hashes = {p.relative_to(source).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
              for p in files(source)}
    digest = hashlib.sha256(json.dumps(hashes, sort_keys=True).encode()).hexdigest()
    return {'sha256': digest, 'files': hashes}


def inventory(source):
    packages = {}
    for path in files(source):
        if path.name != 'package.xml':
            continue
        root = ET.fromstring(path.read_text(encoding='utf-8'))
        name = root.findtext('name')
        if name in packages:
            raise ValueError('Duplicate package: ' + name)
        cmake = path.parent / 'CMakeLists.txt'
        text = cmake.read_text(encoding='utf-8') if cmake.exists() else ''
        text = re.sub(r'#[^\n]*', '', text)
        packages[name] = {
            'path': path.parent.relative_to(source).as_posix(),
            'manifest': {tag: sorted({e.text.strip() for e in root.findall(tag)})
                         for tag in ('depend', 'build_depend', 'buildtool_depend',
                                     'build_export_depend', 'exec_depend', 'test_depend')},
            'cmake_find_package': sorted(set(re.findall(r'find_package\s*\(\s*([\w]+)', text))),
        }
    for item in packages.values():
        deps = set(sum(item['manifest'].values(), [])) | set(item['cmake_find_package'])
        item['source_edges'] = sorted(deps & packages.keys())
        item['external_dependencies'] = sorted(deps - packages.keys())
        item['cmake_not_literal_manifest'] = sorted(set(item['cmake_find_package']) - set(sum(item['manifest'].values(), [])))
    vendor = (source / 'thirdparty/graaf_vendor/CMakeLists.txt').read_text(encoding='utf-8')
    url = re.search(r'GIT_REPOSITORY\s+(\S+)', vendor).group(1)
    tag = re.search(r'GIT_TAG\s+(\S+)', vendor).group(1)
    return {'packages': packages, 'graaf': {'url': url, 'tag': tag,
            'headers_bundled': (source / 'thirdparty/graaf_vendor/include/graaflib').exists()}}


def command(args):
    try:
        result = subprocess.run(args, capture_output=True, text=True, timeout=90)
        return {'argv': args, 'returncode': result.returncode,
                'stdout': result.stdout, 'stderr': result.stderr}
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {'argv': args, 'returncode': None, 'error': str(exc)}


def preflight(source):
    report = inventory(source)
    blockers = []
    report['scope'] = 'read-only; no server/GPU/launch/network/install checks; NOT a build'
    audit = HERE.parent / 'wildos_port/dependency_audit.json'
    if audit.exists():
        prior = json.loads(audit.read_text(encoding='utf-8'))
        report['prior_audit'] = {'sha256': hashlib.sha256(audit.read_bytes()).hexdigest(),
                                 'platform': prior.get('platform'),
                                 'blockers': prior.get('blockers')}
    report['platform'] = platform.platform()
    report['python'] = {'executable': sys.executable, 'version': platform.python_version(),
                        'SOABI': sysconfig.get_config_var('SOABI')}
    report['ROS_DISTRO'] = os.environ.get('ROS_DISTRO')
    release = Path('/etc/os-release')
    report['os_release'] = release.read_text() if release.exists() else None
    if sys.platform != 'linux' or not report['os_release'] or 'VERSION_ID="22.04"' not in report['os_release'] or 'ID=ubuntu\n' not in report['os_release']:
        blockers.append('Target must be Ubuntu 22.04 Linux')
    if os.environ.get('ROS_DISTRO') != 'humble' or not Path('/opt/ros/humble/setup.bash').is_file():
        blockers.append('Missing ROS 2 Humble or unsourced /opt/ros/humble/setup.bash')
    if sys.version_info[:2] != (3, 10):
        blockers.append('Humble Ubuntu binary ABI requires Python 3.10')
    if sys.platform == 'linux' and Path(sys.executable).resolve() != Path('/usr/bin/python3').resolve():
        blockers.append('Use /usr/bin/python3, not a venv/conda interpreter; build pins system Python')
    report['tools'] = {tool: shutil.which(tool) for tool in ('ros2', 'colcon', 'rosdep', 'cmake', 'g++', 'git')}
    report['tool_versions'] = {tool: command([path, '--version'])
                               for tool, path in report['tools'].items()
                               if path and tool in ('cmake', 'g++', 'git', 'rosdep')}
    for tool, path in report['tools'].items():
        if not path:
            blockers.append('Missing tool: ' + tool)
    report['abi_imports'] = command([sys.executable, '-B', '-c',
        'import rclpy; from rclpy.impl.implementation_singleton import rclpy_implementation; import rosidl_generator_py; print(rclpy_implementation.__file__)'])
    if report['abi_imports']['returncode'] != 0:
        blockers.append('ROS Python extension/generator import failed; inspect ABI report')
    if report['tools']['colcon']:
        report['colcon_selection'] = command(['colcon', 'build', '--help'])
        if '--packages-up-to' not in report['colcon_selection'].get('stdout', ''):
            blockers.append('colcon packages-up-to extension unavailable')
    if report['tools']['rosdep']:
        report['rosdep_check'] = command(['rosdep', 'check', '--from-paths', str(source),
                                         '--ignore-src', '--rosdistro', 'humble'])
        if report['rosdep_check']['returncode'] != 0:
            blockers.append('rosdep check failed (missing dependencies, unresolved keys or local database)')
    prefixes = [Path(p) for key in ('AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH')
                for p in os.environ.get(key, '').split(os.pathsep) if p]
    report['ament_presence'] = {}
    dependencies = sorted({d for p in report['packages'].values() for d in p['external_dependencies']})
    for dep in dependencies:
        report['ament_presence'][dep] = [str(p) for p in prefixes
            if (p / 'share/ament_index/resource_index/packages' / dep).is_file()]
    cmake_deps = {d for package in report['packages'].values()
                  for d in package['cmake_find_package']} - report['packages'].keys()
    for dep in sorted(cmake_deps - {'Eigen3', 'OpenCV', 'nanoflann'}):
        if not report['ament_presence'].get(dep):
            blockers.append('Required CMake ROS package not indexed in sourced prefixes: ' + dep)
    # Read config paths only: CMake configure probes can write or fetch content.
    report['system_cmake_configs'] = {}
    roots = [Path('/usr/lib'), Path('/usr/share'), Path('/usr/local/lib'), Path('/usr/local/share')]
    for dep in ('Eigen3', 'OpenCV', 'nanoflann'):
        found = sorted({str(p) for root in roots + prefixes if root.exists()
                        for pattern in (dep + 'Config.cmake', dep.lower() + '-config.cmake')
                        for p in root.rglob(pattern)}) if sys.platform == 'linux' else []
        report['system_cmake_configs'][dep] = found
        if not found:
            blockers.append('CMake config not located: ' + dep + ' (not an apt package name)')
    report['python_distributions'] = {}
    for line in (source / 'requirements.txt').read_text(encoding='utf-8').splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        name = re.split(r'[<>=!~;\[]', line)[0].strip()
        try:
            version = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            version = None
        report['python_distributions'][line] = version
    report['packaging_missing_markers'] = [d + '/__init__.py' for d in
        json.loads((HERE / 'namespace_patch.json').read_text())['directories']
        if not (source / d / '__init__.py').exists()]
    if report['graaf']['url'] != 'https://github.com/bobluppes/graaf.git' or report['graaf']['tag'] != 'v1.1.1':
        blockers.append('Graaf recipe changed; re-audit before using build.sh')
    report['warnings'] = ['Python distributions are runtime inventory, not a GPU requirement or install recipe.',
        'Manifest coverage is incomplete; rosdep success does not prove CMake/link/import success.',
        'Graaf needs explicit source acquisition; preflight performs no download.',
        'Missing Python package markers are repaired ONLY by --prepare in the new copy.']
    report['blockers'] = blockers
    report['status'] = 'BLOCKED' if blockers else 'PREFLIGHT_ONLY_NOT_BUILT'
    return report


def prepare(source, dest):
    source = source.resolve(strict=True)
    dest = dest.expanduser().absolute()
    if dest.is_symlink() or dest.exists():
        raise ValueError('Destination exists; never reuse: ' + str(dest))
    parent = dest.parent.resolve(strict=True)
    dest = parent / dest.name
    home = Path.home().resolve()
    if home not in dest.parents or source == dest or source in dest.parents or dest in source.parents:
        raise ValueError('Destination must be a NEW directory under home, outside source')
    patch = json.loads((HERE / 'namespace_patch.json').read_text())
    setup = (source / 'visual_navigation/setup.py').read_text(encoding='utf-8')
    if "packages=find_packages(exclude=['test'])" not in setup:
        raise ValueError('setup.py changed; re-audit before patching')
    for directory in patch['directories']:
        if not list((source / directory).glob('*.py')):
            raise ValueError('Patch evidence missing: ' + directory)
    original = snapshot(source)
    dest.mkdir(exist_ok=False)
    target = dest / 'src/nebula2-wildos'
    target.mkdir(parents=True)
    for relative in original['files']:
        out = target / relative
        out.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source / relative, out)
        out.chmod((source / relative).stat().st_mode & 0o777 | 0o600)
    (dest / 'original_snapshot.json').write_text(json.dumps(original, indent=2), encoding='utf-8')
    if snapshot(target) != original or snapshot(source) != original:
        raise ValueError('Source changed during copy or copy mismatch; partial destination retained')
    changes = []
    for directory in patch['directories']:
        marker = target / directory / '__init__.py'
        if not marker.exists():
            with marker.open('x', encoding='utf-8', newline='\n') as stream:
                stream.write(patch['payload'])
            changes.append(marker.relative_to(target).as_posix())
    record = {'source': str(source), 'destination': str(dest), 'original_sha256': original['sha256'],
              'added_markers': changes, 'staged_snapshot': snapshot(target),
              'excluded_names': sorted(SKIP), 'excluded_suffixes': sorted(SUFFIXES)}
    (dest / 'prepared.json').write_text(json.dumps(record, indent=2), encoding='utf-8')
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument('--preflight', action='store_true')
    modes.add_argument('--prepare', action='store_true')
    modes.add_argument('--inventory', action='store_true')
    parser.add_argument('--source', type=Path, default=DEFAULT_SOURCE)
    parser.add_argument('--destination', type=Path)
    parser.add_argument('--no-server-precheck', action='store_true', required=True)
    args = parser.parse_args()
    try:
        if args.prepare:
            if args.destination is None:
                parser.error('--prepare requires --destination')
            result = prepare(args.source, args.destination)
        else:
            result = inventory(args.source) if args.inventory else preflight(args.source)
        print(json.dumps(result, indent=2, ensure_ascii=True))
        return 2 if result.get('blockers') else 0
    except (OSError, ValueError, ET.ParseError) as exc:
        print(json.dumps({'status': 'ERROR', 'error': str(exc)}))
        return 1


if __name__ == '__main__':
    sys.exit(main())
