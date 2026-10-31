from pathlib import Path
p=Path('/home/ly/full_score_search_20261015')
s=(p/'analyze_cloud_origin.py').read_text().replace('session_20261015_200533','session_20261015_201924')
(p/'analyze_full_cloud.py').write_text(s)
s=(p/'analyze_sensor_closed.py').read_text().replace('session_20261015_200533','session_20261015_201924').replace("r.glob('field_sensor_audit_search_*')", "r.glob('field_sensor_full_search_*')").replace("out=r/'sensor_audit_20261015_200533.zip'", "out=r/'sensor_full_20261015_201924.zip'")
(p/'analyze_full_closed.py').write_text(s)
s=(p/'analyze_sensor.sh').read_text().replace('analyze_cloud_origin.py','analyze_full_cloud.py').replace('cloud_origin_review.json','full_cloud_origin_review.json').replace('analyze_sensor_closed.py','analyze_full_closed.py').replace('sensor_closed_analysis.json','full_closed_analysis.json')
(p/'analyze_full.sh').write_text(s)
