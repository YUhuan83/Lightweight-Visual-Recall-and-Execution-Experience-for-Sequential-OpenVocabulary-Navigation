"""Request graceful stop only for the console explicitly owned by this diagnostic."""
import os
from pathlib import Path
import signal
import sys
folder=Path(sys.argv[1]).resolve()
assert folder.parent==Path('/home/ly/native_delay_20261018')
pid=int((folder/'console.pid').read_text())
tokens=Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
assert b'/home/ly/native_delay_20261018/timing_console.py' in tokens
os.kill(pid,signal.SIGTERM)
print('Graceful stop requested for diagnostic console',pid)
