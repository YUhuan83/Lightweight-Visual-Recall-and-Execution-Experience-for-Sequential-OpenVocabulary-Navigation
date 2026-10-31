"""Read-only ROS-domain inventory; emit no credentials or unrelated environments."""
import json
from pathlib import Path
rows=[]
for p in Path('/proc').iterdir():
    if not p.name.isdigit():continue
    try:
        env=dict(item.split(b'=',1) for item in (p/'environ').read_bytes().split(b'\0') if b'=' in item)
        domain=env.get(b'ROS_DOMAIN_ID',b'').decode()
        if domain not in ('88','183'):continue
        args=(p/'cmdline').read_bytes().decode(errors='replace').split('\0')
        rows.append(dict(pid=int(p.name),domain=domain,executable=Path(args[0]).name,
                         script=Path(args[1]).name if len(args)>1 else None))
    except (PermissionError,FileNotFoundError,ProcessLookupError):continue
print(json.dumps(rows,indent=2))
