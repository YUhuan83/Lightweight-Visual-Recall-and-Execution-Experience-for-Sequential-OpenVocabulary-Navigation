import sys,hashlib,json
from pathlib import Path
sys.path.insert(0,'/home/ly/frontier_motion_handoff_20261016/candidate/tools')
import server_test_preflight as m
p=Path('/home/ly/frontier_motion_handoff_20261016/field/audit_vision.py')
# Exact diagnostic role identity; original process, DDS and input gates retained.
m.ROLES['vision']=(str(p),)
assert p.is_file()
print(json.dumps(dict(diagnostic_vision=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest())),flush=True)
raise SystemExit(m.main())
