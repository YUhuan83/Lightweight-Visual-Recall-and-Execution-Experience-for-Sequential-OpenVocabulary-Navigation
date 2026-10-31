"""Counterexamples against current production callbacks, no ROS or vehicle.

Passing means the defect is detected; this is not a repair test result.
"""
import json,sys
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock

root=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(root/'nav2_adapter'),str(root/'nav2_adapter/test'),
               str(root/'official_adapter'),str(root/'wildos_port/repro_check/src/nebula2-wildos/visual_navigation')]
from offline_ros_fakes import install
install()
from test_execution_mechanism_ros import ExecutionMechanismTests
from test_graph_path_bridge_routing import drivable_route

fixture=ExecutionMechanismTests();fixture.setUp();node=fixture.node
try:
    node.expected=None
    node.route_endpoint=(10.,0.)
    node.input_freshness_issue=lambda:None
    node.execution_transport_issue=lambda:None
    node.send_motion_segment=Mock()
    node.finish_execution_feedback=Mock()
    node.start_segment=Mock()
    result=[]
    # This same x=0 -> x=1 maneuver can follow an x=1 -> x=0 reversal.
    # The best-ever distance to the unchanged goal remains 9m after each lap.
    for lap in range(3):
        node.motion_queue=[drivable_route((0.,0.),(1.,0.)),drivable_route((1.,0.),(0.,0.))]
        node.odom.pose.pose.position.x=1.
        node.recovery_attempts=3
        node.on_follow_result(Mock(result=Mock(return_value=S(status=4))),node.epoch,(1.,0.))
        result.append(dict(lap=lap,attempts_before=3,attempts_after=node.recovery_attempts,
                           fixed_goal_distance_m=9.,new_best_progress_m=0.))
        assert node.recovery_attempts==0
    # A completed short prefix also clears the budget unconditionally, even
    # though the stable exploration endpoint is still nine metres away.
    node.motion_queue=[drivable_route((0.,0.),(1.,0.))]
    node.recovery_attempts=3
    node.on_follow_result(Mock(result=Mock(return_value=S(status=4))),node.epoch,(1.,0.))
    assert node.recovery_attempts==0
    report=dict(scope=__doc__,same_owner_oscillation=result,
                partial_segment_success_resets_budget=True,
                physical_execution=False,defect_reproduced=True)
    out=Path(__file__).with_suffix('.json');out.write_text(json.dumps(report,indent=2),encoding='utf8')
    print(json.dumps(report,indent=2))
finally:fixture.tearDown()
