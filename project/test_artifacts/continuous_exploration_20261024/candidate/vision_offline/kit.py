"""Download only; evaluate wheel dependency metadata for Linux, never the host."""
import argparse
import email
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import urllib.parse
import urllib.request
import zipfile

from pip._vendor.packaging.markers import default_environment
from pip._vendor.packaging.requirements import Requirement
from pip._vendor.packaging.tags import cpython_tags, compatible_tags
from pip._vendor.packaging.utils import canonicalize_name, parse_wheel_filename
from pip._vendor.packaging.version import Version

ROOT = Path(__file__).resolve().parent
WHEELS = ROOT / 'wheelhouse'
ENV = dict(default_environment(), os_name='posix', sys_platform='linux',
           platform_machine='x86_64', platform_system='Linux',
           platform_release='5.15.0', platform_version='Ubuntu 22.04',
           python_version='3.10', python_full_version='3.10.12',
           implementation_name='cpython', implementation_version='3.10.12',
           platform_python_implementation='CPython', extra='')
PLATFORMS = [f'manylinux_2_{n}_x86_64' for n in range(35, 4, -1)] + [
    'manylinux2014_x86_64', 'manylinux2010_x86_64', 'manylinux1_x86_64', 'linux_x86_64']
TAGS = set(cpython_tags((3, 10), platforms=PLATFORMS)) | set(
    compatible_tags((3, 10), interpreter='cp310', platforms=PLATFORMS))


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(8 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def sanitize(text):
    text = re.sub(r'(https?://)[^\s/@]+:[^\s/@]+@', r'\1[REDACTED]@', text)
    return re.sub(r'(https?://[^\s?#]+)[?#][^\s]+', r'\1[REDACTED_QUERY]', text)


def write_json(name, data):
    (ROOT / name).write_text(json.dumps(data, indent=2, ensure_ascii=True) + '\n', encoding='utf-8')


def inventory():
    result = {}
    for path in sorted(WHEELS.glob('*.whl')):
        name, version, _, tags = parse_wheel_filename(path.name)
        if not tags & TAGS:
            raise ValueError(f'Unsupported target wheel: {path.name}')
        with zipfile.ZipFile(path) as z:
            meta = [n for n in z.namelist() if n.endswith('.dist-info/METADATA') and n.count('/') == 1]
            if len(meta) != 1:
                raise ValueError(f'Invalid metadata: {path.name}')
            m = email.message_from_bytes(z.read(meta[0]))
        if canonicalize_name(m['Name']) != name or Version(m['Version']) != version:
            raise ValueError(f'Metadata identity mismatch: {path.name}')
        if m['Requires-Python']:
            from pip._vendor.packaging.specifiers import SpecifierSet
            if not SpecifierSet(m['Requires-Python']).contains('3.10.12'):
                raise ValueError(f'Unsupported Python: {path.name}')
        if name in result:
            raise ValueError(f'Duplicate package: {name}')
        result[name] = dict(name=name, version=str(version), filename=path.name,
                            bytes=path.stat().st_size, tags=sorted(map(str, tags)),
                            requires_python=m['Requires-Python'],
                            requires_dist=m.get_all('Requires-Dist', []))
    return result


def closure(items, roots):
    pending = list(roots)
    seen = set()
    constraints = {}
    extras = {}
    while pending:
        raw = pending.pop()
        r = Requirement(raw)
        # Dependencies are filtered under their parent's extra before enqueueing.
        name = canonicalize_name(r.name)
        constraints.setdefault(name, set()).add(str(r.specifier))
        current = extras.setdefault(name, {''})
        current.update(r.extras)
        if name not in items:
            continue
        key = (name, tuple(sorted(current)))
        if key in seen:
            continue
        seen.add(key)
        for dep in items[name]['requires_dist']:
            d = Requirement(dep)
            if d.marker is None or any(d.marker.evaluate(dict(ENV, extra=e)) for e in current):
                d.marker = None
                pending.append(str(d))
    missing, conflicts = [], []
    for name, specs in sorted(constraints.items()):
        req = name + ','.join(sorted(s for s in specs if s))
        if name not in items:
            missing.append(req)
        elif not Requirement(req).specifier.contains(items[name]['version']):
            conflicts.append(f'{req}; downloaded={items[name]["version"]}')
    return missing, conflicts, constraints


def roots():
    return [s.strip() for s in (ROOT / 'requirements.in').read_text().splitlines()
            if s.strip() and not s.startswith('#')]


def source(item):
    name, version, filename = item['name'], item['version'], item['filename']
    if name in ('torch', 'torchvision'):
        index = f'https://download.pytorch.org/whl/cu128/{name}/'
        with urllib.request.urlopen(index, timeout=120) as response:
            html = response.read().decode()
        for link in re.findall(r'href="([^"]+)"', html):
            url = urllib.parse.urljoin(index, link)
            parsed = urllib.parse.urlsplit(url)
            if urllib.parse.unquote(parsed.path.rsplit('/', 1)[-1]) == filename:
                sha = urllib.parse.parse_qs(parsed.fragment)['sha256'][0]
                return urllib.parse.urldefrag(url)[0], sha, index
        raise ValueError(f'Official source not found: {filename}')
    index = 'https://pypi.org/simple'
    with urllib.request.urlopen(f'https://pypi.org/pypi/{name}/{version}/json', timeout=120) as response:
        data = json.load(response)
    entry = next(x for x in data['urls'] if x['filename'] == filename)
    return entry['url'], entry['digests']['sha256'], index


def fetch():
    if not ROOT.parent.is_dir() or shutil.disk_usage(ROOT).free < 15 * 1024**3:
        raise RuntimeError('Parent missing or less than 15 GiB free')
    for name in ('wheelhouse', 'cache', 'tmp', 'logs'):
        (ROOT / name).mkdir(exist_ok=True)
    env = dict(os.environ, PIP_CACHE_DIR=str(ROOT / 'cache'),
               TMP=str(ROOT / 'tmp'), TEMP=str(ROOT / 'tmp'),
               PIP_DISABLE_PIP_VERSION_CHECK='1', PYTHONDONTWRITEBYTECODE='1')
    while True:
        items = inventory()
        missing, conflicts, _ = closure(items, roots())
        write_json('status.json', dict(complete=False, wheel_count=len(items),
                   bytes=sum(x['bytes'] for x in items.values()), missing=missing, conflicts=conflicts))
        if conflicts:
            raise RuntimeError(f'Dependency conflicts: {conflicts}')
        if not missing:
            break
        for req in missing:
            name = canonicalize_name(Requirement(req).name)
            index = 'https://download.pytorch.org/whl/cu128' if name in ('torch', 'torchvision') else 'https://pypi.org/simple'
            cmd = [sys.executable, '-m', 'pip', '--isolated', 'download', '--no-deps',
                   '--only-binary=:all:', '--python-version', '3.10', '--implementation', 'cp',
                   '--abi', 'cp310', '--abi', 'abi3', '--abi', 'none', '--dest', str(WHEELS),
                   '--cache-dir', str(ROOT / 'cache'), '--index-url', index,
                   '--timeout', '120', '--retries', '5', '--progress-bar', 'off']
            for platform in PLATFORMS:
                cmd += ['--platform', platform]
            cmd += [req]
            print(f'Downloading {req} from {index}', flush=True)
            proc = subprocess.run(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding='utf-8', errors='replace')
            output = sanitize(proc.stdout)
            (ROOT / 'logs' / f'{name}.log').write_text(output, encoding='utf-8')
            print(output, flush=True)
            if proc.returncode:
                raise RuntimeError(f'pip exit {proc.returncode} for {req}:\n{output}')
    entries = []
    old = ROOT / 'manifest.json'
    prior = {x['filename']: x for x in json.loads(old.read_text())['wheels']} if old.exists() else {}
    for item in items.values():
        print(f'Hash/source verification: {item["filename"]}', flush=True)
        previous = prior.get(item['filename'])
        if previous:
            url, expected, index = previous['source_url'], previous['source_sha256'], previous['source_index']
        else:
            url, expected, index = source(item)
        actual = digest(WHEELS / item['filename'])
        if actual != expected:
            raise ValueError(f'Official SHA256 mismatch: {item["filename"]}')
        entries.append(dict(item, sha256=actual, source_sha256=expected, source_url=url, source_index=index))
    write_json('manifest.json', dict(target=ENV, supported_tags=sorted(map(str, TAGS)), wheels=entries))
    (ROOT / 'requirements.lock').write_text(''.join(
        f'{x["name"]}=={x["version"]} --hash=sha256:{x["sha256"]}\n' for x in entries), encoding='ascii')
    verify()


def verify():
    manifest = json.loads((ROOT / 'manifest.json').read_text())
    items = inventory()
    expected = {x['filename'] for x in manifest['wheels']}
    if expected != {x['filename'] for x in items.values()}:
        raise ValueError('Wheel inventory differs from manifest')
    lock = (ROOT / 'requirements.lock').read_text()
    expected_lock = ''.join(f'{x["name"]}=={x["version"]} --hash=sha256:{x["sha256"]}\n' for x in manifest['wheels'])
    if lock != expected_lock:
        raise ValueError('Lock differs from manifest')
    for entry in manifest['wheels']:
        if digest(WHEELS / entry['filename']) != entry['sha256']:
            raise ValueError(f'Hash mismatch: {entry["filename"]}')
        for key in ('version', 'bytes', 'tags', 'requires_dist', 'requires_python'):
            if items[entry['name']][key] != entry[key]:
                raise ValueError(f'Metadata mismatch: {entry["filename"]}: {key}')
    missing, conflicts, reached = closure(items, roots())
    if missing or conflicts or set(items) != set(reached):
        raise ValueError(f'Closure failure: missing={missing}, conflicts={conflicts}, unreachable={set(items)-set(reached)}')
    status = dict(complete=True, wheel_count=len(items), bytes=sum(x['bytes'] for x in items.values()),
                  missing=[], conflicts=[], target='Ubuntu 22.04 x86_64 CPython 3.10 / CUDA 12.8',
                  checks=['SHA256', 'official source hashes at fetch', 'wheel tags', 'Requires-Python',
                          'recursive Linux markers and extras', 'version constraints', 'lock consistency'])
    write_json('status.json', status)
    print(json.dumps(status, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=['fetch', 'verify'])
    args = parser.parse_args()
    try:
        globals()[args.action]()
    except Exception as exc:
        try:
            items = inventory()
            missing, conflicts, _ = closure(items, roots())
        except Exception as inventory_error:
            items, missing, conflicts = {}, ['Inventory invalid; see error'], [sanitize(str(inventory_error))]
        failure = dict(complete=False, error_type=type(exc).__name__, error=sanitize(str(exc)),
                       wheel_count=len(items), bytes=sum(x['bytes'] for x in items.values()),
                       missing=missing, conflicts=conflicts)
        write_json('status.json', failure)
        print(json.dumps(failure, indent=2), file=sys.stderr)
        sys.exit(1)
