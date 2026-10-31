import hashlib,json
from pathlib import Path
base=Path('/home/ly/execution_continuity_review_20261022');root=base/'v42_retirement_boundary'
for run in (1,2):
    report=json.loads((root/f'real_ros_report_{run}.json').read_text());assert all(r['passed'] for r in report)
    assert '[  PASSED  ] 32 tests.' in (root/f'cpp_after_{run}.log').read_text()
    assert '[  FAILED  ] 3 tests,' in (root/f'cpp_before_{run}.log').read_text()
manifest=json.loads((root/'full/frozen_source_manifest.json').read_text())
assert all(hashlib.sha256((root/'candidate'/n).read_bytes()).hexdigest()==h for n,h in manifest['members'].items())
library=root/'install/wildos_ackermann_rpp/lib/libwildos_ackermann_rpp.so'
config_path=root/'full/config.json';config=json.loads(config_path.read_text())
previous=config['library_sha256']['libwildos_ackermann_rpp.so']
config['library_sha256']['libwildos_ackermann_rpp.so']=hashlib.sha256(library.read_bytes()).hexdigest()
config_path.write_text(json.dumps(config,indent=2))
# Activate the new C++ overlay atomically with this candidate's Python bridge.
runfile=root/'run_full.sh';runtext=runfile.read_text()
old=f'{base}/v37_full_path/install/local_setup.bash'
new=f'{root}/install/local_setup.bash'
assert old in runtext or new in runtext
runfile.write_text(runtext.replace(old,new))
recorder=root/'full/console_graph_capture.py';text=recorder.read_text()
assert text.count('EXTRA_POINT_TOPICS = [')==1
if '"/wildos/controller_path_retirement"' not in text:
    recorder.write_text(text.replace('EXTRA_POINT_TOPICS = [','EXTRA_POINT_TOPICS = ["/wildos/controller_path_retirement", ',1))
receipt=dict(base='v40_observation_heading',candidate=root.name,cpp_tests_per_run=32,cpp_runs=2,
 real_ros_tests=sum(r['tests'] for r in report),previous_rpp_sha256=previous,
 rpp_sha256=config['library_sha256']['libwildos_ackermann_rpp.so'],
 frozen_wrappers={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [config_path,recorder,root/'full/driver_full.py',runfile]},
 search_seconds=180,deadline_origin='actual query_sent.monotonic_ns',acceptance=False,
 note='New bridge and C++ retirement boundary together; full fluent arrival not established')
(root/'trial_frozen.json').write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt,indent=2))
