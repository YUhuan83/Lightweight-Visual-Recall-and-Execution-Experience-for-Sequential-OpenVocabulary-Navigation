"""Stationary live TF/semantic-memory probe; no navigation success claim."""
import json
import time
from pathlib import Path
from beamngpy import BeamNGpy
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String

root = Path('/home/ly/town_memory_validation_20261003')
out = root/'crosswalk_tf100_probe'
out.mkdir(exist_ok=True)
if (out/'probe.json').exists():
    raise RuntimeError('Existing probe evidence must not be overwritten')
rclpy.init()
n = Node('crosswalk_memory_tf_probe')
b = BeamNGpy('127.0.0.1', 25252).open(launch=False)
events = []
started = time.monotonic()
def receive(kind, message):
    events.append(dict(kind=kind, elapsed=time.monotonic()-started,
                       payload=json.loads(message.data)))
subs = [n.create_subscription(String, topic,
          lambda msg, kind=kind: receive(kind, msg), 10)
        for topic, kind in [('/spot1/remembered_object_candidates', 'candidates'),
                            ('/spot1/object_confirmation', 'confirmation')]]
query = n.create_publisher(String, '/spot1/object_search/query', 10)
def spin(seconds):
    end = time.monotonic()+seconds
    while time.monotonic()<end:
        rclpy.spin_once(n, timeout_sec=.05)
def disable(node, name):
    client = n.create_client(SetParameters, node+'/set_parameters')
    if not client.wait_for_service(timeout_sec=5.):
        raise RuntimeError('Missing control parameter service: '+node)
    request = SetParameters.Request(parameters=[Parameter(name=name,
        value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=False))])
    future = client.call_async(request)
    rclpy.spin_until_future_complete(n, future, timeout_sec=5.)
    response = future.result()
    if response is None or not all(r.successful for r in response.results):
        raise RuntimeError('Could not disable '+node)
report = dict(scope=__doc__, initial_query='crosswalk', tf_history_depth=100,
    vision_pid=int((root/'vision.pid').read_text()), passed=False,
    map_reload_count=0, navigation_test=False)
try:
    disable('/wildos_graph_path_bridge', 'follow_path')
    disable('/wildos_cmd_vel_to_control', 'enable')
    spin(.5)
    query.publish(String(data='crosswalk'))
    b.control.resume()
    spin(30.)
    report['station_confirmations_before_query'] = [e for e in events
        if e['kind']=='confirmation' and e['payload'].get('query')=='gas station']
    query.publish(String(data='gas station'))
    spin(8.)
    candidates = [e for e in events if e['kind']=='candidates'
        and e['payload'].get('query')=='gas station']
    report['first_station_candidates'] = candidates[0]['payload'] if candidates else None
    views = candidates[0]['payload'].get('views',[]) if candidates else []
    report['passed'] = bool(views and all(v.get('evidence')=='passive_semantic_landmarks'
        for v in views) and not report['station_confirmations_before_query'])
    report['candidate_count'] = len(views)
except BaseException as exc:
    report['error'] = repr(exc)
finally:
    b.control.pause()
    b.disconnect()
    report['simulation_paused'] = True
    report['events'] = events
    (out/'probe.json').write_text(json.dumps(report, indent=2))
    n.destroy_node()
    rclpy.shutdown()
print(json.dumps({k:v for k,v in report.items() if k!='events'}, indent=2), flush=True)
raise SystemExit(0 if report['passed'] else 1)
