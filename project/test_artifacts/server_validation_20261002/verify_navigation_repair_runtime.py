"""Read deployed control settings and process identities, without driving."""
from pathlib import Path
import json
import subprocess
import sys
sys.path.insert(0, '/home/ly')
from session_tools_20261002 import ROOT, WS, runtime_env
if '--runtime-child' not in sys.argv:
    raise SystemExit(subprocess.run([str(WS/'.venv/bin/python'), __file__, '--runtime-child'],
                                   env=runtime_env()).returncode)
import rclpy
from rcl_interfaces.srv import GetParameters
rclpy.init()
node = rclpy.create_node('navigation_repair_runtime_verify')
result = dict(beamng_pid_preserved=Path('/proc/909616').exists(),
              visual_memory_pid_preserved=Path('/proc/3507782').exists(), parameters={})
try:
    for remote, names in [('/wildos_cmd_vel_to_control', ['max_speed', 'overspeed_limit', 'enable']),
                           ('/wildos_native_vehicle_control', ['overspeed_limit']),
                           ('/wildos_graph_path_bridge', ['follow_path']),
                           ('/controller_server', ['FollowPath.desired_linear_vel']),
                           ('/global_costmap/global_costmap', ['footprint_padding']),
                           ('/local_costmap/local_costmap', ['footprint_padding'])]:
        client = node.create_client(GetParameters, remote+'/get_parameters')
        assert client.wait_for_service(timeout_sec=4), remote
        f = client.call_async(GetParameters.Request(names=names))
        rclpy.spin_until_future_complete(node, f, timeout_sec=4)
        assert f.done() and f.result() is not None, remote
        values = {name: value.bool_value if value.type == 1 else value.double_value
                  for name, value in zip(names, f.result().values)}
        result['parameters'][remote] = values
    p = result['parameters']
    assert p['/wildos_cmd_vel_to_control'] == {'max_speed': 0., 'overspeed_limit': 0., 'enable': False}
    assert p['/wildos_native_vehicle_control']['overspeed_limit'] == 0.
    assert not p['/wildos_graph_path_bridge']['follow_path']
    assert result['beamng_pid_preserved'] and result['visual_memory_pid_preserved']
    result['passed'] = True
    (ROOT/'navigation_repair/runtime.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2), flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
