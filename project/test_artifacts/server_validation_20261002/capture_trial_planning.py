"""Capture held-pose planning inputs without changing the ongoing trial."""
import sys
import subprocess
from pathlib import Path
sys.path.insert(0,'/home/ly')
import session_tools_20261002 as tools
if '--runtime-child' not in sys.argv:
    raise SystemExit(subprocess.run([sys.executable,__file__,sys.argv[1],'--runtime-child'],
        env=tools.runtime_env()).returncode)
environment=tools.ros_environment()
tools.ROOT=Path('/home/ly/full_town_memory_20261002')/sys.argv[1]
tools.ros_environment=lambda: environment
tools.capture_blocked()

