"""Check live measured terrain while driving stays disabled; retain the map."""
from pathlib import Path
import json
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters, GetParameters
from grid_map_msgs.msg import GridMap
from graphnav_msgs.msg import NavigationGraph
from rclpy.serialization import serialize_message
from beamngpy import BeamNGpy

OUT = Path('/home/ly/town_memory_validation_20261004_terrain/live_probe')
OUT.mkdir(exist_ok=True)
rclpy.init()
n = Node('terrain_live_probe')
latest = {}
counts = {}
def received(key, msg):
    if isinstance(msg, GridMap) and not msg.layers:
        counts[key+'_empty'] = counts.get(key+'_empty', 0)+1
        return
    latest[key] = msg
    counts[key] = counts.get(key, 0)+1
for key, topic, typ in [('terrain', '/spot1/local_terrain', GridMap),
                         ('debug', '/spot1/dbg_map', GridMap),
                         ('graph', '/spot1/nav_graph', NavigationGraph)]:
    n.create_subscription(typ, topic, lambda msg, key=key: received(key, msg), 1)

def parameter(remote, name, value):
    client = n.create_client(SetParameters, remote+'/set_parameters')
    assert client.wait_for_service(timeout_sec=5), remote
    request = SetParameters.Request(parameters=[Parameter(name=name, value=ParameterValue(
        type=ParameterType.PARAMETER_BOOL, bool_value=value))])
    future = client.call_async(request)
    rclpy.spin_until_future_complete(n, future, timeout_sec=10)
    assert future.done() and all(r.successful for r in future.result().results), remote
    n.destroy_client(client)

b = None
report = {'driving_enabled': False, 'map_reload_count': 0}
try:
    parameter('/wildos_cmd_vel_to_control', 'enable', False)
    parameter('/wildos_graph_path_bridge', 'follow_path', False)
    client = n.create_client(GetParameters, '/spot1/mapper_node/get_parameters')
    assert client.wait_for_service(timeout_sec=5)
    future = client.call_async(GetParameters.Request(names=['use_occupancy_input']))
    rclpy.spin_until_future_complete(n, future, timeout_sec=10)
    assert future.done()
    report['use_occupancy_input'] = future.result().values[0].bool_value
    assert report['use_occupancy_input'] is False
    n.destroy_client(client)
    b = BeamNGpy('127.0.0.1', 25252, socket_timeout=8).open(launch=False)
    b.control.resume()
    end = time.monotonic()+12
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=.1)
    for key, msg in latest.items():
        (OUT/(key+'.cdr')).write_bytes(serialize_message(msg))
        if isinstance(msg, GridMap):
            layers = {layer: np.array(msg.data[i].data) for i, layer in enumerate(msg.layers)}
            valid = layers.get('elevation_valid', np.zeros(len(layers['elevation']))) > .5
            heights = layers['elevation'][valid]
            report[key] = {'frame': msg.header.frame_id, 'layers': msg.layers,
                           'valid_height_cells': int(valid.sum()),
                           'elevation_min': float(heights.min()) if len(heights) else None,
                           'elevation_max': float(heights.max()) if len(heights) else None,
                           'unknown_fraction': float(np.mean(layers['unknown'] > .5))}
        else:
            report[key] = {'nodes': len(msg.nodes), 'edges': len(msg.edges),
                           'current_node_idx': msg.current_node_idx,
                           'observed_nodes': sum(any(r.confidence >= .7 for r in x.robot_traversability)
                                                 for x in msg.nodes),
                           'infeasible_nodes': sum(any(r.confidence >= .7 and not r.feasible
                                                     for r in x.robot_traversability) for x in msg.nodes)}
    report['counts'] = counts
    assert counts.get('terrain', 0) >= 5 and counts.get('debug', 0) >= 3
    assert report['terrain']['valid_height_cells'] > 20
    report['passed'] = True
except Exception as exc:
    report['passed'] = False
    report['error'] = repr(exc)
    raise
finally:
    if b is not None:
        b.control.pause()
        report['simulation_paused'] = True
        b.disconnect()
    (OUT/'report.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report), flush=True)
    n.destroy_node()
    rclpy.shutdown()
