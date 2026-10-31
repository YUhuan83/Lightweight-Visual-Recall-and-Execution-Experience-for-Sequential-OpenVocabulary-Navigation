from pathlib import Path
repo=Path(r'C:\Users\Yu\Desktop\拟真');root=repo/'test_artifacts/localization_range_20261027';p=root/'candidate/tools/test_physical_sensor_rig.py'
s=(repo/'tools/test_physical_sensor_rig.py').read_text(encoding='utf8').replace('(16, 60., 40., 30, .2)','(16, 60., 80., 30, .2)')
p.write_text(s,encoding='utf8')
