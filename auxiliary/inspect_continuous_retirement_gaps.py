import json,tarfile
from pathlib import Path
root=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts\continuous_exploration_20261024')
with tarfile.open(root/'continuous_r2_compact.tar.gz') as archive:
    name=next(n for n in archive.getnames() if '211456/' in n and n.endswith('/events.jsonl'))
    events=[json.loads(line) for line in archive.extractfile(name).read().decode().splitlines()]
    plans={e['stamp_ns']:e for e in events if e.get('kind')=='plan_status'}
    gaps=json.loads(next(archive.extractfile(n).read() for n in archive.getnames() if '211456/' in n and n.endswith('/command_gaps.json')))
    for gap in gaps[:2]:
        rows=[e for stamp,e in sorted(plans.items()) if gap['from_s']-.8<=stamp/1e9<=gap['to_s']+.4]
        print(json.dumps(dict(gap_s=gap['gap_s'],states=rows),ensure_ascii=False))
