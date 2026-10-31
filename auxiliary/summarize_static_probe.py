from pathlib import Path
import json
p=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts\range_candidate_20261027\diagnostics')
r=json.loads((p/'static_coverage_report.json').read_text(encoding='utf8'))
print('coverage',[(n,sum(x['located'] for x in r['rows'] if x['lines']==n),sum(x['lines']==n for x in r['rows'])) for n in (16,32,64)])
