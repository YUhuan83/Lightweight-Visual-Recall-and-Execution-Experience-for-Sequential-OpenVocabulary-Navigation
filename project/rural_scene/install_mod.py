"""Install only this bundle's native mod into an explicitly selected user folder."""
import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import uuid
import zipfile

ROOT = Path(__file__).resolve().parent
MOD_NAME = 'nizhen_rural.zip'


def digest(path):
    sha = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            sha.update(chunk)
    return sha.hexdigest()


def validate_mod(root):
    source = root / 'generated' / MOD_NAME
    manifest = json.loads((root / 'generated/build_manifest.json').read_text(encoding='utf-8-sig'))
    if not isinstance(manifest, dict) or manifest.get('zip') != MOD_NAME:
        raise ValueError('build_manifest must identify ' + MOD_NAME)
    sha = manifest.get('zip_sha256')
    if not isinstance(sha, str) or not re.fullmatch('[0-9a-f]{64}', sha) or digest(source) != sha:
        raise ValueError('Mod SHA-256 does not match build_manifest')
    if type(manifest.get('zip_bytes')) is not int or source.stat().st_size != manifest['zip_bytes']:
        raise ValueError('Mod byte count does not match build_manifest')
    expected = manifest.get('packaged_files')
    if not isinstance(expected, list) or not expected or any(not isinstance(p, str) for p in expected):
        raise ValueError('build_manifest requires a packaged_files allowlist')
    if len(set(expected)) != len(expected):
        raise ValueError('Duplicate packaged_files entries')
    with zipfile.ZipFile(source) as archive:
        entries = archive.namelist()
        if len(set(entries)) != len(entries) or set(entries) != set(expected):
            raise ValueError('Mod entries do not match the unique packaged_files allowlist')
        for entry in entries:
            if (not entry.startswith(('art/shapes/nizhen_rural/', 'art/prefabs/nizhen_rural/')) or
                    '\\' in entry or ':' in entry or '..' in entry.split('/') or
                    PurePosixPath(entry).as_posix() != entry or entry.endswith('/')):
                raise ValueError('Mod entry is outside the authored native asset roots: ' + entry)
        if archive.testzip() is not None:
            raise ValueError('Mod archive is corrupt')
    return manifest


def installed_mod(user):
    """Accept BeamNG's unpacked form only when authored files match the bundle."""
    validate_mod(ROOT)
    source = ROOT / 'generated' / MOD_NAME
    mods = Path(user).expanduser().resolve() / 'current' / 'mods'
    packed = mods / MOD_NAME
    unpacked = mods / 'unpacked' / Path(MOD_NAME).stem
    if packed.exists() and unpacked.exists():
        raise ValueError('Both packed and unpacked mod exist; resolve duplicate in Mod Manager first')
    if unpacked.exists():
        if unpacked.resolve() != unpacked:
            raise ValueError('Refusing redirected unpacked mod')
        with zipfile.ZipFile(source) as archive:
            for info in archive.infolist():
                target = unpacked / info.filename
                if target.resolve() != target or not target.is_file():
                    raise ValueError('Missing or redirected unpacked asset: ' + info.filename)
                if digest(target) != hashlib.sha256(archive.read(info)).hexdigest():
                    raise ValueError('Unpacked asset differs from bundle: ' + info.filename)
        return unpacked
    if packed.is_file() and packed.resolve() == packed and digest(packed) == digest(source):
        return packed
    raise ValueError('Matching mod not installed; run install_mod.py before starting BeamNG')


def install(user, backup=False):
    validate_mod(ROOT)
    source = ROOT / 'generated' / MOD_NAME
    user = Path(user).expanduser().resolve()
    # BeamNG 0.39 resolves an explicit -userpath to its current/ subdirectory.
    effective = user / 'current'
    if effective.exists() and effective.resolve() != effective:
        raise ValueError('Refusing redirected effective user directory')
    target = effective / 'mods' / MOD_NAME
    if (effective / 'mods' / 'unpacked' / Path(MOD_NAME).stem).exists():
        return installed_mod(user)
    if target.is_symlink() or (target.parent.exists() and target.parent.resolve() != effective / 'mods'):
        raise ValueError('Refusing redirected mod target')
    if target.exists():
        if digest(source) == digest(target):
            return target
        if not backup:
            raise ValueError('Different mod already installed; use --backup to retain it before replacing')
        saved = target.with_name(target.name + '.' + uuid.uuid4().hex + '.bak')
        with saved.open('xb') as stream, target.open('rb') as old:
            shutil.copyfileobj(old, stream)
    target.parent.mkdir(parents=True, exist_ok=True)
    # Exclusive creation avoids overwriting a newly appearing target in normal mode.
    if not backup:
        with target.open('xb') as stream, source.open('rb') as original:
            shutil.copyfileobj(original, stream)
    else:
        temporary = target.with_name('.' + uuid.uuid4().hex + '.tmp')
        try:
            with temporary.open('xb') as stream, source.open('rb') as original:
                shutil.copyfileobj(original, stream)
            temporary.replace(target)
        finally:
            temporary.unlink(missing_ok=True)
    return target


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--user', default=os.environ.get('BEAMNG_USER'))
    parser.add_argument('--backup', action='store_true')
    args = parser.parse_args()
    if not args.user:
        parser.error('--user or BEAMNG_USER is required')
    try:
        print(install(args.user, args.backup))
    except (OSError, ValueError, zipfile.BadZipFile) as exc:
        parser.exit(1, f'Installation refused: {exc}\n')


if __name__ == '__main__':
    main()
