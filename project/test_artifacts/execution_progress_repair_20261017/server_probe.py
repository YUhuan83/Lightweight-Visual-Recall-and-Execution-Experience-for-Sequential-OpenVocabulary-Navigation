"""Read-only /proc residual check and installed planner implementation inventory."""
import hashlib
import json
import os
from pathlib import Path

needles = ('controller_server', 'planner_server', 'beamng_scenario',
    'native_vehicle_control', 'cmd_vel_to_control', 'rviz2', 'ros2 bag record')
rows = []
for p in Path('/proc').iterdir():
    if not p.name.isdigit() or int(p.name) in (os.getpid(), os.getppid()):
        continue
    try:
        cmd = (p/'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        continue
    if any(n in cmd for n in needles):
        rows.append(dict(pid=p.name, cmd=cmd))
headers = []
base = Path('/opt/ros/humble/include/nav2_smac_planner')
if base.exists():
    for p in base.rglob('*'):
        if not p.is_file() or p.suffix not in ('.hpp', '.h'):
            continue
        lines = p.read_text(errors='replace').splitlines()
        hits = [dict(line=i+1, text=line.strip()) for i, line in enumerate(lines)
                if 'change_penalty' in line or 'reverse_penalty' in line]
        if hits:
            headers.append(dict(path=str(p), sha256=hashlib.sha256(p.read_bytes()).hexdigest(), hits=hits))
print(json.dumps(dict(processes=rows, smac_headers=headers), indent=2))
