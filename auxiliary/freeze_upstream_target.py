import hashlib,json
from pathlib import Path
from ament_index_python.packages import get_package_prefix
base=Path('/home/ly/execution_continuity_review_20261022');root=base/'v57_upstream_target_approach'
report=json.loads((root/'real_ros_report.json').read_text());assert all(r['passed'] for r in report)
assert '[  PASSED  ] 32 tests.' in (root/'cpp_preserved.log').read_text()
for i in (1,2):
    assert '[  PASSED  ] 29 tests.' in (base/f'frontier_endpoint_contract_review/after_{i}.log').read_text()
manifest=json.loads((root/'full/frozen_source_manifest.json').read_text())
assert all(hashlib.sha256((root/'candidate'/n).read_bytes()).hexdigest()==h for n,h in manifest['members'].items())
assert Path(get_package_prefix('graphnav_planner'))==base/'v55_frontier_endpoint/install/graphnav_planner'
config_path=root/'full/config.json';config=json.loads(config_path.read_text())
library=base/'v55_frontier_endpoint/install/graphnav_planner/lib/libgraphnav_planner_component.so'
config['library_sha256']['libgraphnav_planner_component.so']=hashlib.sha256(library.read_bytes()).hexdigest()
config_path.write_text(json.dumps(config,indent=2))
receipt=dict(candidate=root.name,source_scope=json.loads((root/'receipt.json').read_text()),real_ros_tests=sum(r['tests'] for r in report),graph_cpp_tests=29,graph_cpp_runs=2,rpp_cpp_tests=32,graph_library_sha256=config['library_sha256']['libgraphnav_planner_component.so'],frozen_wrappers={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [config_path,root/'full/console_graph_capture.py',root/'full/driver_full.py',root/'run_full.sh']},search_seconds=180,acceptance=False,note='Original 3D-target goal entry only; unchanged GraphNav/Smac/RPP/memory/sensor sources. Field pending.')
(root/'trial_frozen.json').write_text(json.dumps(receipt,indent=2))
print(json.dumps(receipt,indent=2))
