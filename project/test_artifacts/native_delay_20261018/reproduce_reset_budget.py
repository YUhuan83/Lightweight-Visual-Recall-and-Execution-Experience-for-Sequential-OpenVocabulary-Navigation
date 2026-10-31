"""Diagnostic counterexample of the field handoff; asserts BUG PRESENT, no repair."""
from pathlib import Path
import json,sys
root=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(root/'nav2_adapter'),str(root/'nav2_adapter/test'),str(root/'official_adapter'),
              str(root/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation')]
from offline_ros_fakes import install
install()
from test_navigation_transactions_ros import NavigationTransactionTests
from types import SimpleNamespace as S
fixture=NavigationTransactionTests();fixture.setUp();steps=[]
try:
    node=fixture.node
    responses=fixture.service()
    responses[-1].set_result(S(success=True))
    steps.append(dict(stage='initial_query_clear',attempts=node.graph_reset_attempts,pending=node.query_reset_pending,successful_services=len(responses)))
    for phase in range(3):
        before=len(responses);node.expected=(10.+phase,0.)
        node.release_visible_approach('visible_target_route_timeout')
        dispatched=len(responses)>before
        if dispatched:responses[-1].set_result(S(success=True))
        steps.append(dict(stage='visible_release_'+str(phase+1),dispatched=dispatched,
                          attempts=node.graph_reset_attempts,pending=node.query_reset_pending,
                          unresolved=len(node.graph_reset_waiting),last_status=node.last_status,
                          successful_services=len(responses)))
    assert [s['dispatched'] for s in steps[1:]]==[True,True,False]
    assert steps[-1]['pending'] and steps[-1]['unresolved']==0
    assert steps[-1]['last_status']['reason']=='response_budget_exhausted'
    report=dict(scope=__doc__,bug_reproduced=True,all_dispatched_services_succeeded=True,
                source='release_visible_approach retains cumulative successful reset attempts',steps=steps,
                production_modified=False,repair_verified=False)
    (Path(__file__).parent/'reset_budget_counterexample.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
finally:fixture.tearDown()
