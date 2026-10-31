"""Read-only runtime mode and raw-goal DDS endpoint check."""
import json,time
from pathlib import Path
import rclpy
from rcl_interfaces.srv import GetParameters
rclpy.init();n=rclpy.create_node('readonly_upstream_approach_interface')
client=n.create_client(GetParameters,'/wildos_graph_path_bridge/get_parameters')
assert client.wait_for_service(timeout_sec=10.)
request=GetParameters.Request();request.names=['upstream_target_approach']
future=client.call_async(request);rclpy.spin_until_future_complete(n,future,timeout_sec=10.)
assert future.done() and future.result().values[0].bool_value is True
until=time.monotonic()+5.
while time.monotonic()<until:
    rclpy.spin_once(n,timeout_sec=.1)
    subs=n.get_subscriptions_info_by_topic('/spot1/raw_object_waypoint')
    pubs=n.get_publishers_info_by_topic('/spot1/raw_object_waypoint')
    if any(x.node_name=='wildos_graph_path_bridge' for x in subs) and pubs:break
report=dict(upstream_target_approach=True,raw_goal_subscribers=[x.node_name for x in subs],
    raw_goal_publishers=[x.node_name for x in pubs],wall_ns=time.time_ns())
assert 'wildos_graph_path_bridge' in report['raw_goal_subscribers'] and pubs
Path('/home/ly/execution_continuity_review_20261022/v57_upstream_target_approach/actual_approach_interface.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2));n.destroy_node();rclpy.shutdown()
