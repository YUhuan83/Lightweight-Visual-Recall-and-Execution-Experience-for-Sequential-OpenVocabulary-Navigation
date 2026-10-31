from pathlib import Path
import json
import shutil

root=Path(__file__).resolve().parents[2]
manifest=json.loads((root/'official_wildos_beamng_bundle.manifest.json').read_text(encoding='utf8'))
frozen=root/'deploy/20261016_recovery_handoff_candidate'
frozen.mkdir(exist_ok=False)
for name in ('official_wildos_beamng_bundle.zip','official_wildos_beamng_bundle.manifest.json','official_wildos_beamng_bundle.sha256'):
    shutil.copy2(root/name,frozen/name)
source=(Path(__file__).parent/'validate_dispatch.sh').read_text(encoding='utf8')
source=source.replace('/home/ly/planning_dispatch_repair_20261016','/home/ly/planning_recovery_handoff_20261016')
source=source.replace('f916c0a04f3c37cf85712ba27bfc5a882401ae8b9f12286e784496d16cc25aa7',manifest['sha256'])
source=source.replace("'nav2_adapter/test/test_graph_path_bridge_routing.py'},diff", "'nav2_adapter/test/test_graph_path_bridge_routing.py','nav2_adapter/test/test_navigation_transactions_ros.py','nav2_adapter/wildos_nav2_adapter/navigation_transactions.py'},diff")
source=source.replace("names=['test_planner_execution_ownership_ros','test_tracking_handoff_ros','test_short_endpoint_handoff_ros']", "names=['test_planner_execution_ownership_ros','test_tracking_handoff_ros','test_short_endpoint_handoff_ros']+['test_navigation_transactions_ros.NavigationTransactionTests.'+n for n in ('test_escape_waits_for_old_terminal_result_and_settled_stop','test_new_query_revokes_escape_intent_during_cancellation_wait','test_expired_or_moved_escape_intent_falls_back_after_stop')]")
(Path(__file__).parent/'validate_recovery_handoff.sh').write_text(source,encoding='utf8',newline='\n')
print(manifest['sha256'])
