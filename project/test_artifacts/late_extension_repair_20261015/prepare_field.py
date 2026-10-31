from pathlib import Path
import json
src=Path('/home/ly/full_score_search_20261015');dst=Path('/home/ly/late_extension_repair_20261015')
for name in ('drive_sensor_full.py','sensor_full_console.sh','sensor_full_console.json','sensor_console.py','record_execution_contract.py'):
 s=(src/name).read_text().replace('/home/ly/full_score_search_20261015','/home/ly/late_extension_repair_20261015')
 if name=='drive_sensor_full.py':s=s.replace('field_sensor_full_','field_late_extension_')
 if name=='sensor_console.py':s=s.replace("'/wildos/points','/wildos/clearing_points'", "'/wildos/full_planned_path','/wildos/points','/wildos/clearing_points'")
 (dst/name).write_text(s)
print('Diagnostic-only owned recorder adds full_planned_path; production source unchanged')
