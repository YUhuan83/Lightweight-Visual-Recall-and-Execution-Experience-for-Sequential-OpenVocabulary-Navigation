"""Apply and verify the controller's reviewed maneuver tolerance."""
from pathlib import Path
import json
import rclpy
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters

ROOT = Path('/home/ly/town_memory_validation_20261003')
rclpy.init()
node = rclpy.create_node('verify_maneuver_tolerance')
try:
    setter = node.create_client(SetParameters, '/controller_server/set_parameters')
    getter = node.create_client(GetParameters, '/controller_server/get_parameters')
    if not setter.wait_for_service(timeout_sec=10) or not getter.wait_for_service(timeout_sec=10):
        raise RuntimeError('Controller parameter services unavailable')
    request = SetParameters.Request()
    request.parameters = [Parameter(name='maneuver_goal_checker.xy_goal_tolerance',
        value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=0.3))]
    future = setter.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10)
    if not future.done():
        raise RuntimeError('Parameter update timed out')
    results = future.result().results
    if not results or not all(r.successful for r in results):
        raise RuntimeError('Parameter update rejected: '+str([r.reason for r in results]))
    check = GetParameters.Request()
    check.names = ['maneuver_goal_checker.xy_goal_tolerance',
                   'maneuver_goal_checker.yaw_goal_tolerance']
    future = getter.call_async(check)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10)
    if not future.done():
        raise RuntimeError('Verification timed out')
    values = [v.double_value for v in future.result().values]
    if abs(values[0]-0.3)>1e-9:
        raise RuntimeError('Controller did not retain the new tolerance')
    record = {'applied':True, 'xy_goal_tolerance_m':values[0],
              'yaw_goal_tolerance_rad':values[1], 'source':'live_controller_parameter_service'}
    (ROOT/'cusp_parameter_update.json').write_text(json.dumps(record,indent=2))
    print(json.dumps(record))
finally:
    node.destroy_node()
    rclpy.shutdown()
