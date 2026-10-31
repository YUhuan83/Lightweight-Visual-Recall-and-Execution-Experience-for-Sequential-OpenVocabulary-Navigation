import hashlib,json
from pathlib import Path
base=Path('/home/ly/execution_continuity_review_20261022');root=base/'v46_approach_feedback';parent=base/'v42_retirement_boundary'
for run in (1,2):
    report=json.loads((root/f'real_ros_report_{run}.json').read_text());assert all(r['passed'] for r in report)
    assert '[  PASSED  ] 32 tests.' in (root/f'cpp_preserved_{run}.log').read_text()
manifest=json.loads((root/'full/frozen_source_manifest.json').read_text())
assert all(hashlib.sha256((root/'candidate'/n).read_bytes()).hexdigest()==h for n,h in manifest['members'].items())
config_path=root/'full/config.json';config=json.loads(config_path.read_text())
library=parent/'install/wildos_ackermann_rpp/lib/libwildos_ackermann_rpp.so'
assert hashlib.sha256(library.read_bytes()).hexdigest()==config['library_sha256']['libwildos_ackermann_rpp.so']
for p in (root/'candidate/ackermann_rpp').rglob('*'):
    if p.is_file():assert p.read_bytes()==(parent/'candidate/ackermann_rpp'/p.relative_to(root/'candidate/ackermann_rpp')).read_bytes()
receipt=dict(base="v45_approach_contract",cpp_base=parent.name,candidate=root.name,cpp_tests_per_run=32,cpp_runs=2,
    real_ros_tests=sum(r['tests'] for r in report),rpp_sha256=config['library_sha256']['libwildos_ackermann_rpp.so'],
    frozen_wrappers={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [config_path,root/'full/console_graph_capture.py',root/'full/driver_full.py',root/'run_full.sh']},
    search_seconds=180,deadline_origin='actual query_sent.monotonic_ns',acceptance=False,
    note='Accepted approach failure correlated by live instance rather than newest proposed endpoint; full vehicle acceptance pending')
(root/'trial_frozen.json').write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt,indent=2))
