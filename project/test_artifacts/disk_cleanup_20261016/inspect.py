from pathlib import Path
import json
import os
import zipfile

root = Path('/home/ly')
result = {'disk': list(os.statvfs(root)), 'archives': []}
for path in sorted((root / 'wildos-console').glob('*.zip')):
    if path.stat().st_size < 700_000_000:
        continue
    with zipfile.ZipFile(path) as archive:
        members = [i for i in archive.infolist() if not i.is_dir()]
        missing = []
        for item in members:
            target = root / item.filename
            if not target.is_relative_to(root) or not target.is_file() or target.stat().st_size != item.file_size:
                missing.append(item.filename)
        result['archives'].append(dict(path=str(path), size=path.stat().st_size,
            member_count=len(members), examples=[i.filename for i in members[:5]],
            missing_or_different=missing[:10], mismatch_count=len(missing)))
print(json.dumps(result, indent=2))
