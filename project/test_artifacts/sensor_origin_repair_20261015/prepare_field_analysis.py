from pathlib import Path
p=Path('/home/ly/full_score_search_20261015');o=Path('/home/ly/sensor_origin_repair_20261015')
s=(p/'analyze_full_cloud.py').read_text().replace('session_20261015_201924','session_20261015_203918')
(o/'analyze_field_cloud.py').write_text(s)
s=(p/'analyze_full_closed.py').read_text().replace('session_20261015_201924','session_20261015_203918').replace("out=r/'sensor_full_20261015_201924.zip'","out=r/'sensor_origin_field_20261015_203918.zip'")
s=s.replace("out=r/'sensor_origin_field_20261015_203918.zip'", "files.update(x for x in o.glob('*') if x.is_file())\nout=r/'sensor_origin_field_20261015_203918.zip'") if False else s
s=s.replace("files.update(p for p in Path('/home/ly/full_score_search_20261015').glob('*') if p.is_file())", "files.update(p for p in Path('/home/ly/full_score_search_20261015').glob('*') if p.is_file())\nfiles.update(p for p in Path('/home/ly/sensor_origin_repair_20261015').glob('*') if p.is_file())".replace('\n','
'))
# Give diagnostic members from each directory distinct archive paths.
s=s.replace("else 'diagnostics/'+p.name", "else 'diagnostics/'+p.parent.name+'/'+p.name")
(o/'analyze_field_closed.py').write_text(s)
s=(p/'analyze_full.sh').read_text().replace('/home/ly/full_score_search_20261015/analyze_full_cloud.py','/home/ly/sensor_origin_repair_20261015/analyze_field_cloud.py').replace('/home/ly/full_score_search_20261015/full_cloud_origin_review.json','/home/ly/sensor_origin_repair_20261015/field_cloud_origin_review.json').replace('/home/ly/full_score_search_20261015/analyze_full_closed.py','/home/ly/sensor_origin_repair_20261015/analyze_field_closed.py').replace('/home/ly/full_score_search_20261015/full_closed_analysis.json','/home/ly/sensor_origin_repair_20261015/field_closed_analysis.json')
(o/'analyze_field.sh').write_text(s)
s=(p/'progress.py').read_text().replace('session_20261015_163334','session_20261015_203918');(o/'progress.py').write_text(s)
