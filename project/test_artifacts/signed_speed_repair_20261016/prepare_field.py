from pathlib import Path
src=Path('/home/ly/late_extension_repair_20261015');dst=Path('/home/ly/signed_speed_repair_20261016')
for name in ('drive_sensor_full.py','sensor_full_console.sh','sensor_full_console.json','sensor_console.py','record_execution_contract.py'):
 s=(src/name).read_text().replace('/home/ly/late_extension_repair_20261015','/home/ly/signed_speed_repair_20261016')
 if name=='drive_sensor_full.py':s=s.replace('field_late_extension_','field_signed_speed_')
 (dst/name).write_text(s)
print('Full search prepared; full candidate path recording retained')
