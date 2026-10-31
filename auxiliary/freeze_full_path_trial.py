import hashlib,json
from pathlib import Path
root=Path('/home/ly/execution_continuity_review_20261022/v37_full_path')
report=json.loads((root/'real_ros_report.json').read_text())
assert all(r['passed'] for r in report)
manifest=json.loads((root/'full/frozen_source_manifest.json').read_text())
assert all(hashlib.sha256((root/'candidate'/n).read_bytes()).hexdigest()==h for n,h in manifest['members'].items())
library=root/'install/wildos_ackermann_rpp/lib/libwildos_ackermann_rpp.so'
assert library.is_file()
config_path=root/'full/config.json';config=json.loads(config_path.read_text())
previous=config['library_sha256']['libwildos_ackermann_rpp.so']
config['library_sha256']['libwildos_ackermann_rpp.so']=hashlib.sha256(library.read_bytes()).hexdigest()
config_path.write_text(json.dumps(config,indent=2))
recorder=root/'full/console_graph_capture.py';text=recorder.read_text()
old='EXTRA_POINT_TOPICS = [';assert text.count(old)==1
if '"/wildos/controller_path_progress"' not in text:
    recorder.write_text(text.replace(old,'EXTRA_POINT_TOPICS = ["/wildos/controller_path_progress", ',1))
receipt=dict(base='v35_confirmed_vehicle',candidate='v37_full_path',
    cpp_tests_per_run=26,cpp_runs=2,real_ros_tests=sum(r['tests'] for r in report),
    previous_rpp_sha256=previous,new_rpp_sha256=config['library_sha256']['libwildos_ackermann_rpp.so'],
    full_wrapper_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (config_path,recorder,root/'full/driver_full.py')},
    goal='green residential house',search_seconds=180,startup_outside_budget=True,
    scope='Actual plugins and callbacks passed; full field acceptance not yet established')
(root/'trial_frozen.json').write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt,indent=2))
