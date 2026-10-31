import sys,hashlib,json
from pathlib import Path
sys.path.insert(0,'/home/ly/wildos-console/bundle/tools')
import server_test_preflight as m
p=Path('/home/ly/search_mechanism_audit_20261015/audit_vision.py')
# Explicit diagnostic role identity, all count/DDS/input gates remain unchanged.
m.ROLES['vision']=(str(p),)
assert p.is_file()
print(json.dumps(dict(diagnostic_vision=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest())),flush=True)
raise SystemExit(m.main())
