import difflib,hashlib,json,zipfile
from pathlib import Path
root=Path(r'C:\Users\Yu\Desktop\拟真\test_artifacts\arrival_decoupling_20261025')
candidate=root/'candidate'
manifest=json.loads((candidate/'_frozen_source_manifest.json').read_text(encoding='utf8'))
allowed={
    'nav2_adapter/wildos_nav2_adapter/arrival_criteria.py',
    'nav2_adapter/wildos_nav2_adapter/graph_path_bridge.py',
    'nav2_adapter/wildos_nav2_adapter/runtime_contract.py',
    'official_adapter/beamng_wildos_adapter/goal_standoff.py',
    'nav2_adapter/test/test_goal_confirmation_ros.py',
    'nav2_adapter/test/test_execution_mechanism_ros.py',
    'nav2_adapter/test/offline_ros_fakes.py',
    'nav2_adapter/test/test_graph_path_bridge_routing.py',
    'nav2_adapter/test/test_surface_association_ros.py',
    'nav2_adapter/test/test_multi_goal_session_ros.py',
    'tools/offline_callback_check.py',
}
added={'nav2_adapter/test/test_body_arrival_ros.py'}
changed={name for name,sha in manifest['members'].items() if hashlib.sha256((candidate/name).read_bytes()).hexdigest()!=sha}
assert changed==allowed,(changed-allowed,allowed-changed)
with zipfile.ZipFile(root/'arrival_v25_source.zip') as base,zipfile.ZipFile(root/'body_arrival_delta.zip','w',zipfile.ZIP_DEFLATED) as z:
    z.writestr('base_hashes.json',json.dumps({n:manifest['members'][n] for n in changed},indent=2))
    for name in sorted(changed|added):
        z.write(candidate/name,name)
        if name in changed:
            diff=''.join(difflib.unified_diff(base.read(name).decode('utf8').replace('\r\n','\n').splitlines(True),(candidate/name).read_text(encoding='utf8').splitlines(True),fromfile='v25/'+name,tofile='v26/'+name))
            (root/(name.replace('/','__')+'.diff')).write_text(diff,encoding='utf8')
receipt=dict(base='/home/ly/execution_continuity_review_20261022/v25_fast_shift',
    target='/home/ly/execution_continuity_review_20261022/v26_body_arrival',
    archive_sha256=hashlib.sha256((root/'body_arrival_delta.zip').read_bytes()).hexdigest(),
    changed={n:hashlib.sha256((candidate/n).read_bytes()).hexdigest() for n in sorted(changed|added)},
    body_distance_m=2.0,arrival_basis='vehicle_body_clearance',
    scope='Only arrival measurement, geometric-confirmation/approach separation, evidence matching and related regressions. Frozen motion, planning, sensors, depth thresholds and four-observation consensus unchanged.')
(root/'delta_receipt.json').write_text(json.dumps(receipt,indent=2),encoding='utf8')
print(json.dumps(receipt,indent=2))
