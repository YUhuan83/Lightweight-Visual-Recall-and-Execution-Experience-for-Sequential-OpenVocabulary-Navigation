"""Read-only ZIP inventory and streamed CRC check; no extraction or key reads."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import stat
import zipfile


def unsafe_name(name):
    normalized = name.replace('\\', '/')
    path = PurePosixPath(normalized)
    return (path.is_absolute() or '..' in path.parts
            or bool(re.match(r'^[A-Za-z]:', normalized)) or '\x00' in name)


def inspect(path, verify=False):
    report = dict(archive=path.name, bytes=path.stat().st_size,
                  inspection='read_only_no_extraction', crc_mode='not_requested',
                  license_contents_read=False, executables=[], examples=[],
                  sensitive_files_present=False, unsafe_entries=[], files=0,
                  uncompressed_bytes=0, linux_binary_present=False,
                  windows_binary_present=False, runtime_verified=False)
    with zipfile.ZipFile(path) as archive:
        infos = archive.infolist()
        seen = set()
        blocked = []
        for info in infos:
            if info.is_dir():
                continue
            report['files'] += 1
            report['uncompressed_bytes'] += info.file_size
            name = info.filename.replace('\\', '/')
            lower = name.lower()
            mode = (info.external_attr >> 16) & 0xFFFF
            if unsafe_name(name) or lower in seen or stat.S_ISLNK(mode):
                blocked.append(name)
            seen.add(lower)
            sensitive = PurePosixPath(lower).name == 'tech.key' or lower.endswith(('.key', '.pem'))
            if sensitive:
                report['sensitive_files_present'] = True
            if lower.endswith('binlinux/beamng.tech.x64'):
                report['linux_binary_present'] = True
                report['executables'].append(name)
            if lower.endswith('bin64/beamng.tech.x64.exe'):
                report['windows_binary_present'] = True
                report['executables'].append(name)
            if not sensitive and ('/tech/docker/' in '/'+lower or
                                  ('/tech/' in '/'+lower and any(s in lower for s in ('readme','ros','example')))):
                report['examples'].append(name)
        report['unsafe_entries'] = blocked
        if verify and blocked:
            raise ValueError('Unsafe, duplicate or symlink entries: CRC pass refused; no extraction performed')
        if verify:
            # ZipExtFile validates each file CRC when fully consumed. Skip secrets.
            skipped = 0
            for info in infos:
                lower = info.filename.lower()
                if info.is_dir():
                    continue
                if PurePosixPath(lower.replace('\\','/')).name == 'tech.key' or lower.endswith(('.key','.pem')):
                    skipped += 1
                    continue
                with archive.open(info) as stream:
                    while stream.read(8 * 1024 * 1024):
                        pass
            report['crc_mode'] = 'passed_non_secret_entries'
            report['crc_skipped_secret_files'] = skipped
    if verify and not report['sensitive_files_present']:
        digest = hashlib.sha256()
        with path.open('rb') as stream:
            while chunk := stream.read(8 * 1024 * 1024):
                digest.update(chunk)
        report['sha256'] = digest.hexdigest()
        report['hash_note'] = 'Local fingerprint only; not publisher authenticity verification.'
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('archive', type=Path)
    parser.add_argument('--verify-crc', action='store_true', help='Full streamed pass; can take many minutes')
    args = parser.parse_args()
    try:
        if not args.archive.is_file() or args.archive.suffix.lower() != '.zip':
            raise ValueError('Expected a completed local .zip file; URLs are not accepted')
        print(json.dumps(inspect(args.archive,args.verify_crc),indent=2,ensure_ascii=False))
    except (OSError,ValueError,zipfile.BadZipFile,RuntimeError) as exc:
        print(json.dumps({'status':'failed','reason':str(exc)},ensure_ascii=False))
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
