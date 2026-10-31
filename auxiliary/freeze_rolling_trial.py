import hashlib,json
from pathlib import Path
base=Path('/home/ly/execution_continuity_review_20261022');root=base/'v38_rolling_observation';parent=base/'v37_full_path'
report=json.loads((root/'real_ros_report.json').read_text());assert all(r['passed'] for r in report)
manifest=json.loads((root/'full/frozen_source_manifest.json').read_text())
assert all(hashlib.sha256((root/'candidate'/n).read_bytes()).hexdigest()==h for n,h in manifest['members'].items())
config=json.loads((root/'full/config.json').read_text())
library=parent/'install/wildos_ackermann_rpp/lib/libwildos_ackermann_rpp.so'
assert hashlib.sha256(library.read_bytes()).hexdigest()==config['library_sha256']['libwildos_ackermann_rpp.so']
for name in ('cpp_preserved_1.log','cpp_preserved_2.log'):assert '[  PASSED  ] 26 tests.' in (root/name).read_text()
geo='wildos_port/repro_check/src/nebula2-wildos/visual_navigation/visual_navigation/explorfm_triangulation/localization_geometry.py'
assert (root/'candidate'/geo).read_bytes()==(parent/'candidate'/geo).read_bytes()
receipt=dict(base='v37_full_path',candidate=root.name,cpp_tests_per_run=26,cpp_runs=2,
    real_ros_tests=sum(r['tests'] for r in report),rpp_sha256=config['library_sha256']['libwildos_ackermann_rpp.so'],
    geometry_sha256=hashlib.sha256((root/'candidate'/geo).read_bytes()).hexdigest(),
    search_seconds=180,deadline_origin='actual query_sent.monotonic_ns',
    frozen_wrappers={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [root/'full/config.json',root/'full/driver_full.py',root/'full/console_graph_capture.py']},
    acceptance=False,note='Do not promote if relevant field metrics regress; complete target search pending')
(root/'trial_frozen.json').write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt,indent=2))
