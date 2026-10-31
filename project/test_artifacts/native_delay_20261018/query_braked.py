"""Activate visual workload while speed-loop/FollowPath gates stay closed."""
import json
from pathlib import Path
import sys,time
import rclpy
from std_msgs.msg import String
from rcl_interfaces.srv import GetParameters
rclpy.init();node=rclpy.create_node('native_delay_braked_query_load')
report=dict(passed=False);rows=[]
try:
    deadline=time.monotonic()+180
    while not any(i.node_name=='wildos' for i in node.get_subscriptions_info_by_topic('/spot1/object_search/query')):
        if time.monotonic()>deadline:raise TimeoutError('vision ready')
        rclpy.spin_once(node,timeout_sec=.1)
    for owner,param in [('wildos_cmd_vel_to_control','enable'),('wildos_graph_path_bridge','follow_path')]:
        client=node.create_client(GetParameters,'/'+owner+'/get_parameters')
        assert client.wait_for_service(timeout_sec=3)
        future=client.call_async(GetParameters.Request(names=[param]))
        rclpy.spin_until_future_complete(node,future,timeout_sec=3)
        assert future.done() and not future.result().values[0].bool_value,owner+' gate must stay closed'
    started=time.time_ns()
    node.create_subscription(String,'/spot1/remembered_object_candidates',lambda m:rows.append(json.loads(m.data)),10)
    pub=node.create_publisher(String,'/spot1/object_search/query',10)
    until=time.monotonic()+2
    while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
    pub.publish(String(data='green residential house'))
    deadline=time.monotonic()+15
    while not any(r.get('query')=='green residential house' and r.get('issued_at_ns',0)>=started for r in rows):
        if time.monotonic()>deadline:raise TimeoutError('vision query ACK')
        rclpy.spin_once(node,timeout_sec=.05)
    report.update(passed=True,query_ack=rows[-1],actuation_gates_closed=True)
except BaseException as exc:report['error']=repr(exc)
finally:
    Path(sys.argv[1]).write_text(json.dumps(report,indent=2)+'\n')
    node.destroy_node();rclpy.shutdown()
sys.exit(0 if report['passed'] else 1)
