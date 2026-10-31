from pathlib import Path
import json,hashlib,ast
out=Path('/home/ly/full_score_search_20261015')
source=Path('/home/ly/town_memory_validation_20261014/curvature_contract_candidate1/record_execution_contract.py')
s=source.read_text();assert s.count('time.monotonic()-began<125')==1
s=s.replace('time.monotonic()-began<125','time.monotonic()-began<900')
ast.parse(s);(out/'record_execution_contract.py').write_text(s)
c=json.loads(Path('/home/ly/wildos-console/audit_console.json').read_text());c['goal_timeout_s']=420
(out/'full_console.json').write_text(json.dumps(c,indent=2))
for p in out.glob('*.py'):ast.parse(p.read_text())
print(json.dumps({'config':c,'diagnostic_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in out.iterdir() if p.is_file()}},indent=2))
