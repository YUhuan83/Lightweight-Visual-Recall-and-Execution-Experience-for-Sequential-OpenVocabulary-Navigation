"""Explicit user-owned cleanup with closed evidence; no project evidence removal."""
from pathlib import Path
import hashlib
import json
import os
import shutil
import sys
import time

ROOT = Path('/home/ly').resolve(strict=True)
OUT = ROOT / 'disk_cleanup_20261016'
OUT.mkdir(exist_ok=True)
ARCHIVE = ROOT / 'beamng-downloads/BeamNG.tech.v0.39.4.0.zip'
OLD = ROOT / 'beamng-install-xyHm9iQX'
CURRENT = ROOT / 'beamng-install-0.39.4.0'

def safe(path):
    resolved = path.resolve(strict=True)
    assert resolved.is_relative_to(ROOT) and resolved != ROOT
    assert not path.is_symlink() and path.stat().st_uid == os.getuid()
    return resolved

def digest(path):
    with path.open('rb') as stream:
        result = hashlib.sha256()
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            result.update(block)
        return result.hexdigest()

def disk():
    return dict(zip(('total', 'used', 'free'), shutil.disk_usage(ROOT)))

def write(name, data):
    (OUT / name).write_text(json.dumps(data, indent=2))
    print(json.dumps(data), flush=True)

if sys.argv[1] == 'archive':
    before = disk()
    target = safe(ARCHIVE)
    current = safe(CURRENT)
    assert (current / 'BeamNG.tech.v0.39.4.0/BinLinux/BeamNG.tech.x64').is_file()
    assert target.is_file() and target.stat().st_size == 22982395117
    size = target.stat().st_size
    target.unlink()
    write('archive_deleted.json', dict(time=time.time(), path=str(target),
        bytes=size, before=before, after=disk(), reason='Redownloadable installer; active installed engine preserved'))
elif sys.argv[1] == 'compare':
    old, current = safe(OLD), safe(CURRENT)
    result = dict(old=str(old), current=str(current), files=[], differences=[], bytes=0)
    for path in sorted(old.rglob('*')):
        if path.is_symlink():
            result['differences'].append(dict(path=str(path), reason='symlink'))
            continue
        if not path.is_file():
            continue
        safe(path)
        relative = path.relative_to(old)
        other = current / relative
        size = path.stat().st_size
        if not other.is_file() or other.is_symlink() or other.stat().st_size != size:
            result['differences'].append(dict(path=str(relative), reason='missing_or_size'))
            continue
        sha = digest(path)
        if sha != digest(other):
            result['differences'].append(dict(path=str(relative), reason='hash'))
            continue
        result['files'].append(dict(path=str(relative), size=size, sha256=sha))
        result['bytes'] += size
    result['closed_at'] = time.time()
    write('old_install_compare.json', result)
elif sys.argv[1] == 'old_install_verified_files':
    result = json.loads((OUT / 'old_install_compare.json').read_text())
    old, current = safe(OLD), safe(CURRENT)
    assert result['old'] == str(old) and result['current'] == str(current)
    # Rehash both files immediately before each unlink; no wall-clock TTL is
    # needed for this mode (the shared host clock may jump during simulation).
    before = disk()
    deleted = []
    for item in result['files']:
        target, preserved = safe(old / item['path']), safe(current / item['path'])
        assert target.is_relative_to(old) and preserved.is_relative_to(current)
        assert target.is_file() and preserved.is_file()
        assert target.stat().st_size == preserved.stat().st_size == item['size']
        assert digest(target) == digest(preserved) == item['sha256']
        target.unlink()
        deleted.append(item)
        write('verified_files_deleted.json', dict(time=time.time(), root=str(old),
            deleted=deleted, bytes=sum(x['size'] for x in deleted),
            differences_preserved=result['differences'], before=before, after=disk()))
elif sys.argv[1] == 'old_install':
    result = json.loads((OUT / 'old_install_compare.json').read_text())
    old = safe(OLD)
    assert not result['differences'] and result['old'] == str(old)
    assert result['current'] == str(safe(CURRENT)) and time.time()-result['closed_at'] < 3600
    expected = {f['path']:f['size'] for f in result['files']}
    actual = {}
    for path in old.rglob('*'):
        assert not path.is_symlink()
        if path.is_file():
            safe(path)
            actual[str(path.relative_to(old))] = path.stat().st_size
    assert actual == expected
    # Exact, resolved target was verified above; never infer a deletion path.
    before = disk()
    shutil.rmtree(old)
    write('old_install_deleted.json', dict(time=time.time(), path=str(old),
        files=len(expected), bytes=result['bytes'], before=before, after=disk(),
        reason='All old files SHA256-identical to preserved current engine'))
else:
    raise ValueError(sys.argv[1])
