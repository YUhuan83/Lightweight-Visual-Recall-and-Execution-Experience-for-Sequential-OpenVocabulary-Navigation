"""Freeze source/test delta and the joint probe over the preserved 6153 archive."""
from pathlib import Path
import hashlib,json,shutil,zipfile
out=Path(__file__).resolve().parent;root=out.parents[1]
files=['nav2_adapter/wildos_nav2_adapter/'+p+'.py' for p in
       ('navigation_transactions','graph_path_bridge','execution_recovery','visible_approach')]
files+=['nav2_adapter/test/'+p+'.py' for p in ('test_navigation_transactions_ros','test_execution_mechanism_ros',
    'test_graph_path_bridge_routing','test_multi_goal_session_ros')]
files+=['wildos_port/repro_check/src/nebula2-wildos/graphnav_planner/src/planner_node.cpp',
    'tools/server_test_preflight.py','tools/test_source_control_preflight.py']
files+=['test_artifacts/goal_release_repair_20261019/'+p for p in ('run_regression.py','probe_joint.py')]
manifest=dict(baseline_bundle='6153ebb78acdd7af7b8473d347da2b7363e97d769105fdb4f2261517bfe92d5b',
    members={p:hashlib.sha256((root/p).read_bytes()).hexdigest() for p in files},production_deployment=False,field_acceptance=False)
with zipfile.ZipFile(out/'candidate_delta.zip','x',zipfile.ZIP_DEFLATED) as z:
    for p in files:z.write(root/p,p)
    z.writestr('candidate_delta.json',json.dumps(manifest,indent=2)+'\n')
(out/'candidate_delta.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf8')
shutil.copyfile(root/'test_artifacts/offline_navigation/callback_report.json',out/'full_callback_report.json')
print(json.dumps(dict(files=len(files),**manifest),indent=2))
