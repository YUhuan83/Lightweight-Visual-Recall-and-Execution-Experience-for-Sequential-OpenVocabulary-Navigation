"""Remove two already-authorized old recordings from the evidence archive."""
from pathlib import Path
import json
import sys
import zipfile

root = Path(sys.argv[1]).resolve(strict=True)
archive = root / 'evidence_20261002.zip'
if archive.is_symlink() or archive.resolve().parent != root:
    raise RuntimeError('Unexpected archive path')
if not archive.exists():
    print(json.dumps({'archive_exists': False}))
    raise SystemExit(0)
authorized = {'house_attempt4/front_camera.mp4', 'house_attempt7/front_camera.mp4'}
temporary = root / 'evidence_20261002.without_old_videos.tmp'
if temporary.exists():
    raise RuntimeError('Unexpected existing temporary archive')
with zipfile.ZipFile(archive) as source:
    removed = [i.filename for i in source.infolist() if i.filename in authorized]
    if removed:
        with zipfile.ZipFile(temporary, 'w') as destination:
            destination.comment = source.comment
            for entry in source.infolist():
                if entry.filename not in authorized:
                    destination.writestr(entry, source.read(entry.filename))
if removed:
    with zipfile.ZipFile(temporary) as check:
        if check.testzip() is not None:
            raise RuntimeError('Rewritten archive failed CRC verification')
    temporary.replace(archive)
    (root / 'deleted_archived_videos_20261002.json').write_text(
        json.dumps({'archive': str(archive), 'removed': removed}, indent=2), encoding='utf-8')
with zipfile.ZipFile(archive) as check:
    remaining = [n for n in check.namelist() if n.lower().endswith(('.mp4', '.avi', '.webm', '.mkv'))]
print(json.dumps({'removed': removed, 'remaining_video_entries': remaining}))
