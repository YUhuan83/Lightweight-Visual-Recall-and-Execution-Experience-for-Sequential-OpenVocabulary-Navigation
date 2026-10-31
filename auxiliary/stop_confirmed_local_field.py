"""Gracefully stop only the live v56 field driver; it owns /quit cleanup."""
import os,signal
from pathlib import Path
root=Path('/home/ly/execution_continuity_review_20261022/v56_confirmed_local_continuity')
found=[]
for p in Path('/proc').iterdir():
    if not p.name.isdigit():continue
    try:
        args=(p/'cmdline').read_bytes().split(b'\0')
        if b'full/driver_full.py' in args and (p/'cwd').resolve()==root:found.append(int(p.name))
    except (OSError,RuntimeError):continue
assert len(found)==1,found
os.kill(found[0],signal.SIGTERM)
print('Sent normal shutdown request to v56 field driver:',found[0])
