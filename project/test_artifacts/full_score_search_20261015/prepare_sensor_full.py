from pathlib import Path
import json
p=Path('/home/ly/full_score_search_20261015')
c=json.loads((p/'sensor_console.json').read_text());c['goal_timeout_s']=420
(p/'sensor_full_console.json').write_text(json.dumps(c,indent=2))
s=(p/'sensor_console.sh').read_text().replace('/sensor_console.json','/sensor_full_console.json')
(p/'sensor_full_console.sh').write_text(s)
s=(p/'drive_sensor.py').read_text().replace('/sensor_console.sh','/sensor_full_console.sh').replace('field_sensor_audit_','field_sensor_full_')
(p/'drive_sensor_full.py').write_text(s)
