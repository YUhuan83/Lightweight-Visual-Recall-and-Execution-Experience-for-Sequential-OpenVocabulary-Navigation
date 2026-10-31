from pathlib import Path
import json
p=Path('/home/ly/full_score_search_20261015')
c=json.loads((p/'full_console.json').read_text());c['goal_timeout_s']=180
(p/'sensor_console.json').write_text(json.dumps(c,indent=2))
