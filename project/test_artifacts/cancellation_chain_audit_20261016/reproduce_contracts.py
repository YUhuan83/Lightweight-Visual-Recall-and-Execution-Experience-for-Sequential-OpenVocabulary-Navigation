"""Mechanism counterexamples using unchanged production callbacks and ROS doubles."""
from pathlib import Path
import json,sys,time
from unittest.mock import Mock
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).parent
sys.path[:0]=[str(ROOT/'nav2_adapter'),str(ROOT/'nav2_adapter/test')]
from offline_ros_fakes import install
install()
import rclpy
from std_msgs.msg import String
from test_graph_path_bridge_routing import ExplorationTargetHandoffTests

rclpy.init();results=[]
try:
    for reason,recovering in [('frontier_resolved',False),('vehicle_route_rejected',False),
                              ('frontier_resolved',True),('vehicle_route_rejected',True),
                              ('graph_frame_changed',True)]:
        fixture=ExplorationTargetHandoffTests();fixture.setUp();node=fixture.node
        try:
            node.escape_trace.append((0.,0.,0.,time.monotonic()))
            node.planning_escape_failures.append(('old-transaction',0.,0.,time.monotonic()))
            node.recovering=recovering
            node.following=True
            handle=node.follow_handle=Mock()
            node.follow_active=dict(serial=7,handle=handle)
            message=json.loads(fixture.retirement().data);message['reason']=reason
            node.on_exploration_retired(String(data=json.dumps(message)))
            results.append(dict(reason=reason,independent_recovery_before=recovering,
                cancel_requests=handle.cancel_goal_async.call_count,
                preserved_corridor_points=len(node.escape_trace),
                preserved_planning_failures=len(node.planning_escape_failures),
                independent_recovery_after=node.recovering,quarantined=7 in node.follow_quarantine,
                last_status=node.last_status['status']))
        finally:fixture.tearDown()
finally:rclpy.shutdown()
assert results[0]['cancel_requests']==1 and results[0]['preserved_corridor_points']==0
assert results[1]['preserved_corridor_points']==1
assert results[2]['cancel_requests']==1 and not results[2]['independent_recovery_after']
assert results[3]['cancel_requests']==0 and results[3]['independent_recovery_after']
assert results[4]['preserved_corridor_points']==0 and not results[4]['independent_recovery_after']
report=dict(scope='Fault mechanisms reproduced on unchanged actual production callbacks with ROS doubles; not a repair, DDS or driving acceptance',
    reproduced=True,cases=results,
    faults=['Healthy action cancellation follows benign frontier retirement',
            'New same-frame reason falls through to full physical-history reset',
            'Independent recovery is canceled by that same incomplete classification'])
(OUT/'mechanism_reproduction.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf8')
print(json.dumps(report,indent=2))
