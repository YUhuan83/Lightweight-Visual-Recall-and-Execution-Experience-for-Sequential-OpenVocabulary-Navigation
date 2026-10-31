import os,signal
from pathlib import Path
for pid,fragment in ((3048500,b'/v9/full/console_graph_capture.py'),(3047488,b'full/driver_full.py')):
    path=Path(f'/proc/{pid}')
    if not path.exists():continue
    assert fragment in (path/'cmdline').read_bytes(),pid
    assert str((path/'cwd').resolve()).startswith('/home/ly/execution_continuity_review_20261022/v9'),pid
    os.kill(pid,signal.SIGINT)
    print('SIGINT',pid)
