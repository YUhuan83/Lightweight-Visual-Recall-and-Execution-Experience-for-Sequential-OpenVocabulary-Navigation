import json
from pathlib import Path
p=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts\depth_geometry_20261027\field')
r=json.loads((p/'retirement_boundary_audit.json').read_text(encoding='utf8'))
print('retirement',len(r['fences']),sum(len(f['future_branch_violations']) for f in r['fences']))
s=json.loads((p/'continuous_path_audit.json').read_text(encoding='utf8'))
print('keys',list(s))
print({k:v for k,v in s.items() if any(w in k for w in ('curvature','rotation','violation'))})
