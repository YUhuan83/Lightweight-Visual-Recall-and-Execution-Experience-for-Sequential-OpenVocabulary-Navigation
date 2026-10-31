import json
from pathlib import Path
p=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts\range_candidate_20261027\diagnostics');a=p/'range_candidate_replay_1.json';b=p/'range_candidate_replay_2.json';assert a.read_bytes()==b.read_bytes()
r=json.loads(a.read_text(encoding='utf8'));print('range', [x for x in r['proposals'] if x['evidence']=='range_candidate']);print('delivery',r['range_measurements'])
