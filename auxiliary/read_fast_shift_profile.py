import json
from pathlib import Path
import rclpy
from rcl_interfaces.srv import GetParameters
rclpy.init()
node=rclpy.create_node('read_only_shift_profile')
client=node.create_client(GetParameters,'/wildos_cmd_vel_to_control/get_parameters')
try:
    assert client.wait_for_service(timeout_sec=8.)
    names=['gear_change_confirmation_time','direction_change_speed','command_timeout','state_timeout','require_stamped_command']
    future=client.call_async(GetParameters.Request(names=names))
    rclpy.spin_until_future_complete(node,future,timeout_sec=8.)
    assert future.done() and not future.exception()
    result={n:(v.bool_value if v.type==1 else v.double_value) for n,v in zip(names,future.result().values)}
    assert result['gear_change_confirmation_time']==.3 and result['direction_change_speed']==.15,result
    Path('/home/ly/execution_continuity_review_20261022/v25_fast_shift/loaded_shift_profile.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result))
finally:
    node.destroy_node();rclpy.shutdown()
